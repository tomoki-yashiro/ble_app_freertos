#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "app_util_platform.h"
#include "nrf_soc.h"
#include "nrf_sdh_soc.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_ppi.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "app_timer.h"

#include "timeslot_advertiser.h"

#define NRF_LOG_MODULE_NAME TS_ADV

#if TIMESLOT_ADVERTISER_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL   TIMESLOT_ADVERTISER_CONFIG_LOG_LEVEL
#else
#define NRF_LOG_LEVEL   0
#endif

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/* ToDo: 値確認 */
#define APP_SOC_OBSERVER_DEFAULT_PRIORITY 0

#define HFCLK                   NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED
#define TIMESLOT_LENGTH         4300  /* 4.3ms */
#define TIMESLOT_TIMEOUT        10000 /* 10ms */

#define ADDR_TYPE_OFFSET        (0)
#define ADDR_TYPE_MASK          (0x40)
#define ADDR_TYPE_PUBLIC        (0x00)
#define ADDR_TYPE_RANDOM        (0x40)

#define PDU_TYPE_OFFSET         (0)
#define PDU_TYPE_MASK           (0x0F)

#define SIZE_OFFSET             (1)

#define PAYLOAD_OFFSET          (TS_ADV_ADDR_OFFSET + BLE_GAP_ADDR_LEN)

#define SCAN_REQUEST_TIMEOUT    (200) /* 200us */

#define ADV_INTERVAL_TICKS_ADJUSTMENT   (200)

typedef enum{
    ACTION_SEND_ADV,
    ACTION_RECV_SCAN_REQ,
    ACTION_SEND_SCAN_RSP,
    ACTION_IDLE,
} ts_radio_action_t;

/*****************************************************************************/

static ts_adv_packet_get_func_t packet_get_func;

static ts_adv_packet_t adv_packet;
static ts_adv_packet_t rsp_packet;
static bool response;

/* Scan Request 受信用バッファ */
static uint8_t rx_buffer[TS_ADV_PACKET_SIZE];

static ts_radio_action_t radio_next_action;

static uint8_t radio_channel_map;
static uint8_t radio_channel;
static int8_t tx_power;

static nrf_ppi_channel_t ppi_channel;

/* advertisement interval */
static uint32_t interval_us;
static uint32_t max_delay_us;
static uint32_t interval_ticks;
#if 0
static uint32_t last_start_ticks;
#endif

static uint32_t delay_rng_pool[256];
static uint8_t pool_index = 0;  /* uint8_t なので [0-255] の範囲。
                                   delay_rng_pool の範囲を越える事はない。 */

static bool adv_requested;

static app_timer_t delay_request_timer;

static nrf_radio_request_t req_earliest =
    {NRF_RADIO_REQ_TYPE_EARLIEST,
     .params.earliest = {
            .hfclk      = HFCLK,
            .priority   = 0, /* ts_adv_params_set() で設定する */
            .length_us  = TIMESLOT_LENGTH,
            .timeout_us = TIMESLOT_TIMEOUT,
        },
    };

static nrf_radio_request_t req_normal =
    {NRF_RADIO_REQ_TYPE_NORMAL,
     .params.normal = {
            .hfclk       = HFCLK,
            .priority    = 0, /* ts_adv_params_set() で設定する */
            .distance_us = 0, /* 実行時に設定する */
            .length_us   = TIMESLOT_LENGTH,
        },
    };

/*****************************************************************************
 * Static Functions
 *****************************************************************************/

static __INLINE void radio_channel_iterate(void)
{
    while(((radio_channel_map & (1 << (++radio_channel - 37))) == 0) &&
          radio_channel < 40);
}

static bool is_scan_req_for_me(void)
{
    /* Check CRC */
    if(0 == NRF_RADIO->CRCSTATUS){
        return false;
    }

    /* Check message type and length */
    if((rx_buffer[PDU_TYPE_OFFSET] & PDU_TYPE_MASK) != PDU_TYPE_SCAN_REQ ||
       rx_buffer[SIZE_OFFSET] != BLE_GAP_ADDR_LEN * 2){
        return false;
    }

    /* Check addr */
    return (memcmp(&adv_packet.packet[TS_ADV_ADDR_OFFSET],
                   &rx_buffer[PAYLOAD_OFFSET], BLE_GAP_ADDR_LEN) == 0);
}

static void delay_request_timer_handler(void* p_context)
{
    if(adv_requested){
        /* あえてエラーチェックしない */
        sd_radio_request(&req_earliest);
    }
}

/*****************************************************************************
 * NRF_TIMER0 関連
 *****************************************************************************/

static __INLINE void timer_setup(void)
{
    /* タイマ停止 */
    NRF_TIMER0->TASKS_STOP = 1;

    /* Timer mode */
    NRF_TIMER0->MODE = 0;

    /* 16MHz を 1MHz にする。つまり usec 単位になる。 */
    NRF_TIMER0->PRESCALER = 4;

    /* 16bit。 PRESCALER の設定と合わせて 65535us までのタイマになる。*/
    NRF_TIMER0->BITMODE = 0;

    /* 全イベントをクリア */
    NRF_TIMER0->EVENTS_COMPARE[0] =
        NRF_TIMER0->EVENTS_COMPARE[1] =
        NRF_TIMER0->EVENTS_COMPARE[2] =
        NRF_TIMER0->EVENTS_COMPARE[3] =
        NRF_TIMER0->EVENTS_COMPARE[4] =
        NRF_TIMER0->EVENTS_COMPARE[5] = 0;

    /* ショートカットクリア */
    NRF_TIMER0->SHORTS = 0;

    /* 割り込みを全部無効 */
    NRF_TIMER0->INTENCLR = ((1 << TIMER_INTENSET_COMPARE0_Pos) |
                            (1 << TIMER_INTENSET_COMPARE1_Pos) |
                            (1 << TIMER_INTENSET_COMPARE2_Pos) |
                            (1 << TIMER_INTENSET_COMPARE3_Pos) |
                            (1 << TIMER_INTENSET_COMPARE4_Pos) |
                            (1 << TIMER_INTENSET_COMPARE5_Pos));

    NVIC_EnableIRQ(TIMER0_IRQn);
}

static __INLINE void timer_start(uint32_t value)
{
    /* COMPARE0 イベントをクリア */
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;

    /* タイムアウト値 */
    NRF_TIMER0->CC[0] = value;

    /* COMPARE0 の割り込みを有効 */
    NRF_TIMER0->INTENSET = (1 << TIMER_INTENSET_COMPARE0_Pos);

    /* タイマスタート */
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
}

static __INLINE void timer_stop(void)
{
    /* タイマ停止 */
    NRF_TIMER0->TASKS_STOP = 1;

    /* COMPARE0 イベントをクリア */
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;

    /* COMPARE0 割り込みを無効 */
    NRF_TIMER0->INTENCLR = (1 << TIMER_INTENCLR_COMPARE0_Pos);
}

/*******************************************************************************
 * NRF_RADIO 関連
 ******************************************************************************/
static __INLINE void radio_setup(void)
{
    /* Reset all states in the radio peripheral */
    NRF_RADIO->POWER = 1;

    NRF_RADIO->SHORTS = 0;

    NRF_RADIO->EVENTS_DISABLED = 0;

    /* Set radio configuration parameters */
    NRF_RADIO->TXPOWER = (tx_power << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE    = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos);

    /* Access Address は 0x8e89bed6
       Bluetooth Specification 4.2 Vol 6, Part B, 2.1.2 Access Address
       NRF_RADIO->PCNF1 で NRF_RADIO->BASE0 の長さを 3 に設定している。
       送信も受信も logical address 0 (AP0 + BASE0) を使う。*/
    NRF_RADIO->PREFIX0     =
        (((0x8e) << RADIO_PREFIX0_AP0_Pos) & RADIO_PREFIX0_AP0_Msk);
    NRF_RADIO->BASE0       = 0x89bed600;
    NRF_RADIO->TXADDRESS   = 0x00;
    NRF_RADIO->RXADDRESSES = 0x01;

    /* Bluetooth Specification 4.2 Vol 6, Part B, Figure 2.1
       Preamble は 8ビット

       Bluetooth Specification 4.2 Vol 6, Part B, Figure 2.3
       S0 は Figure 2.3 の PDU Type, RFU, TxAdd, RxAdd に相当し1バイト。
       length は Figure 2.3 の Length に相当し 6ビット(ビット指定に注意)
       S1 は Figure 2.3 の MSB 側 RFU で 2ビット(ビット指定に注意) */
    NRF_RADIO->PCNF0 =
        (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) |
        (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk) |
        (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |
        (((RADIO_PCNF0_PLEN_8bit) << RADIO_PCNF0_PLEN_Pos) & RADIO_PCNF0_PLEN_Msk);

    /* PDU の最大は 39バイト。
       Bluetooth Specification 4.2 Vol 6, Part B, 2.1 PACKET FORMAT
       NRF_RADIO->PCNF1 で指定する最大長は S1 より後の長さで、BT Spec
       の PDU header は含まないので 37。

       Data whitening は有効
       Bluetooth Specification 4.2 Vol 6, Part B, 3.2 DATA WHITENING */
    NRF_RADIO->PCNF1 =
        (((37UL) << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  |
        (((0UL)  << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
        (((3UL)  << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)   |
        (((RADIO_PCNF1_ENDIAN_Little)   << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)  |
        (((RADIO_PCNF1_WHITEEN_Enabled) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);

    /* CRC 関連
       Bluetooth Specification 4.2 Vol 6, Part B, 2.1.4 CRC
       Bluetooth Specification 4.2 Vol 6, Part B, 3.1.1 CRC Generation */
    NRF_RADIO->CRCCNF  =
        (RADIO_CRCCNF_LEN_Three     << RADIO_CRCCNF_LEN_Pos) | // 24 bit
        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // CRC の対象は PDU のみ
    NRF_RADIO->CRCINIT = 0x555555;
    /* Polynomial は X^24 + x^10 + X^9 + X^6 + X^4 + X^3 + x + 1 で 24
       bit だから 0000 0000 0000 01110 0101 1011 = 0x00065B ということ?
       X^24 の扱いは? */
    NRF_RADIO->CRCPOLY = 0x00065B;

    /* T_IFS は 150us
       Bluetooth Specification 4.2 Vol 6, Part B, 4.1 INTER FRAME SPACE */
    NRF_RADIO->TIFS = 150;

    /* Enable radio interrupt propagation */
    NVIC_EnableIRQ(RADIO_IRQn);
}

static void radio_send_adv(uint8_t channel, uint8_t * p_packet, bool response)
{
    static const uint8_t freq[] = {
        2,                      /* 2402 MHz */
        26,                     /* 2426 MHz */
        80,                     /* 2480 MHz */
    };

    ASSERT(channel >= 37 && channel <= 39);

    NRF_RADIO->FREQUENCY = freq[channel - 37];
    NRF_RADIO->DATAWHITEIV = channel;

    /* TXEN タスクを実行 */
    NRF_RADIO->TASKS_TXEN = 1;

    NRF_RADIO->PACKETPTR = (uint32_t)p_packet;

    if(response){
        /* ショートカットで READY -> START -> 送信 -> END -> DISABLED
           までを自動で遷移するように設定。かつ、DISABLED になったら、
           Scan Request に備えて RXEN タスクを実行するように設定。 */
        NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                             RADIO_SHORTS_END_DISABLE_Msk |
                             RADIO_SHORTS_DISABLED_RXEN_Msk);

        radio_next_action = ACTION_RECV_SCAN_REQ;

    }else{
        /* ショートカットで READY -> START -> 送信 -> END -> DISABLED
           までを自動で遷移するように設定。 */
        NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                             RADIO_SHORTS_END_DISABLE_Msk);

        radio_next_action = ACTION_IDLE;
    }

    /* DISABLED イベントでの割り込みを有効。つまり、送信が完了した時の
       割り込み。 */
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
}

static __INLINE void radio_receive_scan_req(uint8_t * p_packet)
{
    NRF_RADIO->PACKETPTR = (uint32_t)p_packet;

    /* ショートカットで READY -> START -> 受信 -> END -> DISABLED まで
       を自動で遷移するように設定。かつ、DISABLED になったら、Scan
       Response に備えて TXEN タスクを実行するように設定。 */
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                         RADIO_SHORTS_END_DISABLE_Msk |
                         RADIO_SHORTS_DISABLED_TXEN_Msk);

    /* DISABLED イベントでの割り込みを有効。つまり、受信が完了した時の
       割り込み。 */
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

    radio_next_action = ACTION_SEND_SCAN_RSP;
}

static __INLINE void radio_send_scan_rsp(uint8_t * p_packet)
{
    NRF_RADIO->PACKETPTR = (uint32_t)p_packet;

    /* ショートカットで READY -> START -> 送信 -> END -> DISABLED まで
       を自動で遷移するように設定。 */
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                         RADIO_SHORTS_END_DISABLE_Msk);

    /* DISABLED イベントでの割り込みを有効。つまり、送信が完了した時の
       割り込み。 */
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;

    radio_next_action = ACTION_IDLE;
}

static __INLINE void radio_clear_disabled_event(void)
{
    /* DISABLED の割り込みを無効 */
    NRF_RADIO->INTENCLR = RADIO_INTENCLR_DISABLED_Msk;

    /* DISABLED イベントをクリア */
    NRF_RADIO->EVENTS_DISABLED = 0;
}

static __INLINE void radio_request_send_adv(void)
{
    /* ショートカットは全て無効 */
    NRF_RADIO->SHORTS = 0;

    /* DISABLED 割り込みを有効にした後、自ら DISABLE タスクを実行。*/
    NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
    NRF_RADIO->TASKS_DISABLE = 1;

    radio_next_action = ACTION_SEND_ADV;
}

/*******************************************************************************
 ******************************************************************************/
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    static nrf_radio_signal_callback_return_param_t return_param;

    /* default action */
    return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

    switch(sig){
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        if(adv_requested){
            timer_setup();
            radio_setup();

            /* NRF_RADIO->EVENTS_ADDRESS でタイマを停止させる ppi。
               Scan Request 待ちのタイムアウトの解除用。 */
            nrf_ppi_channel_endpoint_setup(ppi_channel,
                                           (uint32_t)&NRF_RADIO->EVENTS_ADDRESS,
                                           (uint32_t)&NRF_TIMER0->TASKS_STOP);

            /* アドバタイズパケット取得 */
            response = packet_get_func(&adv_packet, &rsp_packet);

#if 0
            last_start_ticks = app_timer_cnt_get();
#endif

            radio_channel = 36;     /* radio_channel_iterate() されるので 36 */
            radio_request_send_adv();

        }else{
            /* 停止がリクエストされていたら、Timeslot を終了 */
            return_param.callback_action =
                NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
        }
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO: /* NRF_RADIO での割り込み */

        /* DISABLED での割り込みかをチェック */
        if(NRF_RADIO->EVENTS_DISABLED == 0){
            /* DISABLED 以外での割り込みでは何もしない。 */
            NRF_LOG_ERROR("Unexpected interrupt for NRF_RADIO");
            break;
        }
        radio_clear_disabled_event();

        switch(radio_next_action){
        case ACTION_SEND_ADV:
            radio_channel_iterate();
            if(radio_channel < 40){
                /* 1チャネル分だけ送信 */
                radio_send_adv(radio_channel, adv_packet.packet, response);

            }else{
                /* 全チャネル送信した */

                if(adv_requested){
                    /* 次の Timeslot をリクエストして Timeslot を終了 */
                    req_normal.params.normal.distance_us =
                        interval_us + delay_rng_pool[pool_index++] % max_delay_us;
                    return_param.params.request.p_next = &req_normal;
                    return_param.callback_action =
                        NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;

                }else{
                    /* 停止がリクエストされていたら、Timeslot を終了 */
                    return_param.callback_action =
                        NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
                }
            }
            break;

        case ACTION_RECV_SCAN_REQ:
            /* Scan Request を待つ */
            radio_receive_scan_req(rx_buffer);

            /* Scan Request 待ちのタイムアウト設定 */
            timer_start(SCAN_REQUEST_TIMEOUT);

            nrf_ppi_channel_enable(ppi_channel);
            break;

        case ACTION_SEND_SCAN_RSP:
            /* Scan Request 待ちのタイムアウトを停止 */
            timer_stop();
            nrf_ppi_channel_disable(ppi_channel);

            if(is_scan_req_for_me()){
                /* 自分宛の Scan Request なら Scan Response を送信 */
                radio_send_scan_rsp(rsp_packet.packet);
                break;
            }
            /* fall through */

        case ACTION_IDLE:
            radio_request_send_adv();
            break;

        default:
            break;
        }
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0: /* NRF_TIMER0 での割り込み */

        /* COMPARE0 での割込みかチェック */
        if(NRF_TIMER0->EVENTS_COMPARE[0] == 0){
            /* COMPARE0 以外での割り込みでは何もしない。 */
            NRF_LOG_ERROR("Unexpected interrupt for NRF_TIMER0");
            break;
        }
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        switch(radio_next_action){
        case ACTION_SEND_SCAN_RSP:
            /* Scan Request を受信しなかった */

            timer_stop();
            nrf_ppi_channel_disable(ppi_channel);

            radio_clear_disabled_event();
            radio_request_send_adv();
            break;

        default:
            NRF_LOG_ERROR("Unexpected CC0 interrupt for radio_action=%d",
                          radio_next_action);
            break;
        }
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
    default:
        NRF_LOG_ERROR("Unexpected radio signal %d", sig);
        break;
    }

    return &return_param;
}

static void soc_observer(uint32_t event, void *p_context)
{
    switch(event){
    case NRF_EVT_RADIO_BLOCKED:
        NRF_LOG_DEBUG("NRF_EVT_RADIO_BLOCKED");
        break;
    case NRF_EVT_RADIO_CANCELED:
        NRF_LOG_DEBUG("NRF_EVT_RADIO_CANCELED");
        break;
    case NRF_EVT_RADIO_SESSION_IDLE:
        NRF_LOG_DEBUG("NRF_EVT_RADIO_SESSION_IDLE");
        break;

    case NRF_EVT_RADIO_SESSION_CLOSED:
        NRF_LOG_WARNING("NRF_EVT_RADIO_SESSION_CLOSED");
        return;
    case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        NRF_LOG_WARNING("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN");
        return;
    default:
        return;
    }

    /* NRF_EVT_RADIO_BLOCKED, NRF_EVT_RADIO_CANCELED,
       NRF_EVT_RADIO_SESSION_IDLE のいずれかが通知されてたら、Timeslot
       は終了した状態になっている。 */

#if 0
    if(adv_requested){

        /* 前回のアドバタイズからの経過時間 */
        uint32_t delta_ticks =
            (app_timer_cnt_get() - last_start_ticks) & 0x00ffffff;

        if(APP_TIMER_MIN_TIMEOUT_TICKS + delta_ticks <= interval_ticks){
            /* 次のアドバタイズ開始まで時間がある場合は、
               sd_radio_request() のタイミングを遅らせる。*/

            uint32_t err_code;

            err_code = app_timer_start(&delay_request_timer,
                                       interval_ticks - delta_ticks,
                                       NULL);
            APP_ERROR_CHECK(err_code);

        }else{
            /* 次のアドバタイズ開始まで時間が短い場合は、即
               sd_radio_request() 実行。*/

            /* あえてエラーチェックしない */
            sd_radio_request(&req_earliest);
        }
    }
#endif
}

/*****************************************************************************
 * Interface Functions
 *****************************************************************************/

uint32_t ts_adv_init(ts_adv_config_t const* p_config)
{
    ASSERT(p_config != NULL);
    ASSERT(p_config->packet_get_func != NULL);

    ret_code_t err_code;

    /* 使用するモジュールの初期化 */
    err_code = nrf_drv_ppi_init();
    if(err_code != NRF_SUCCESS &&
       err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED){
        return err_code;
    }

    err_code = nrf_drv_rng_init(NULL);
    if(err_code != NRF_SUCCESS &&
       err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED){
        return err_code;
    }

    /* PPI  */
    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    if(err_code != NRF_SUCCESS){
        return err_code;
    }

    app_timer_id_t tid = &delay_request_timer;
    err_code = app_timer_create(&tid, APP_TIMER_MODE_SINGLE_SHOT,
                                delay_request_timer_handler);
    if(err_code != NRF_SUCCESS){
        return err_code;
    }

    err_code = sd_radio_session_open(&radio_signal_callback);
    if(err_code != NRF_SUCCESS){
        return err_code;
    }

    packet_get_func = p_config->packet_get_func;

    /* 乱数生成 */
    nrf_drv_rng_block_rand((void*)delay_rng_pool, sizeof(delay_rng_pool));

    /* ts_adv_param_set() を呼ばないと、ts_adv_start() できないように。 */
    interval_us = 0;

    NRF_SDH_SOC_OBSERVER(m_ts_sys_obs,
                         APP_SOC_OBSERVER_DEFAULT_PRIORITY,
                         soc_observer,
                         NULL);

    return NRF_SUCCESS;
}

uint32_t ts_adv_params_set(ts_adv_params_t const* p_params)
{
    ASSERT(p_params != NULL);

    if(p_params->interval_us == 0){
        return NRF_ERROR_INVALID_PARAM;
    }

    uint32_t channel_map =
        ((p_params->channel_mask[4] & 0x10)? 0x00: 0x01) | /* ch 37 */
        ((p_params->channel_mask[4] & 0x20)? 0x00: 0x02) | /* ch 38 */
        ((p_params->channel_mask[4] & 0x40)? 0x00: 0x04);  /* ch 39 */
    if(channel_map == 0){
        return NRF_ERROR_INVALID_PARAM;
    }

    /* !!!重要!!!: 対応している TX Power は SDK(SoftDevice?) のバージョ
       ンによって変わる場合がある。SDK(SoftDevice?) のバージョンを変更
       した場合にはサポートしている TX Power を必ずチェックし、変更が
       あれば修正すること。*/
    switch((uint8_t)p_params->tx_power){
    case RADIO_TXPOWER_TXPOWER_Pos4dBm:
    case RADIO_TXPOWER_TXPOWER_Pos3dBm:
    case RADIO_TXPOWER_TXPOWER_0dBm:
    case RADIO_TXPOWER_TXPOWER_Neg4dBm:
    case RADIO_TXPOWER_TXPOWER_Neg8dBm:
    case RADIO_TXPOWER_TXPOWER_Neg12dBm:
    case RADIO_TXPOWER_TXPOWER_Neg16dBm:
    case RADIO_TXPOWER_TXPOWER_Neg20dBm:
    case RADIO_TXPOWER_TXPOWER_Neg40dBm:
        /* ok */
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    switch(p_params->priority){
    case NRF_RADIO_PRIORITY_HIGH:
    case NRF_RADIO_PRIORITY_NORMAL:
        /* ok */
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    /* parameter check all OK */

    CRITICAL_REGION_ENTER();

    interval_us = p_params->interval_us;

    max_delay_us = (p_params->max_delay_us > 0)?
        p_params->max_delay_us: 1;

    interval_ticks = APP_TIMER_TICKS(interval_us / 1000);
    if(interval_ticks > ADV_INTERVAL_TICKS_ADJUSTMENT){
        interval_ticks -= ADV_INTERVAL_TICKS_ADJUSTMENT;
    }

    radio_channel_map = channel_map;

    tx_power = p_params->tx_power;

    req_earliest.params.earliest.priority = p_params->priority;
    req_normal.params.normal.priority = p_params->priority;

    CRITICAL_REGION_EXIT();

    NRF_LOG_INFO("interval=%d max_delay=%d channel_map=0x%02X tx_power=%d priority=%d",
                 interval_us,
                 max_delay_us,
                 radio_channel_map,
                 tx_power,
                 req_earliest.params.earliest.priority);

    return NRF_SUCCESS;
}

uint32_t ts_adv_start(void)
{
    NRF_LOG_INFO("ts_adv_start()");

    if(interval_us == 0){
        /* ts_adv_param_set() not called */
        return NRF_ERROR_INVALID_STATE;
    }

    adv_requested = true;

    /* あえてエラーチェックしない */
    app_timer_stop(&delay_request_timer);
    sd_radio_request(&req_earliest);

    return NRF_SUCCESS;
}

uint32_t ts_adv_stop(void)
{
    NRF_LOG_INFO("ts_adv_stop()");

    adv_requested = false;

    /* あえてエラーチェックしない */
    app_timer_stop(&delay_request_timer);

    return NRF_SUCCESS;
}

void ts_adv_packet_setup(ts_adv_packet_t * p_packet,
                         ts_adv_packet_config_t const* p_config)
{
    ASSERT(p_packet != NULL);
    ASSERT(p_config != NULL);

    /* パケットデータ書き換え中に、packet_get_func() を呼び出さないよ
       うに割り込み禁止にしておく。 */

    CRITICAL_REGION_ENTER();

    memset(p_packet, 0, sizeof(ts_adv_packet_t));

    /* addr type */
    p_packet->packet[ADDR_TYPE_OFFSET] &= ~ADDR_TYPE_MASK;
    p_packet->packet[ADDR_TYPE_OFFSET] |=
        (p_config->addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)?
        ADDR_TYPE_PUBLIC: ADDR_TYPE_RANDOM;

    /* addr */
    memcpy(&p_packet->packet[TS_ADV_ADDR_OFFSET],
           &p_config->addr.addr[0],
           BLE_GAP_ADDR_LEN);

    /* pdu type */
    p_packet->packet[PDU_TYPE_OFFSET] &= ~PDU_TYPE_MASK;
    p_packet->packet[PDU_TYPE_OFFSET] |= p_config->pdu_type & PDU_TYPE_MASK;

    /* payload */
    memcpy(&p_packet->packet[PAYLOAD_OFFSET],
           p_config->payload, p_config->payload_length);

    /* length */
    p_packet->packet[SIZE_OFFSET] = BLE_GAP_ADDR_LEN + p_config->payload_length;

    CRITICAL_REGION_EXIT();
}

