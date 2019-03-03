/* Copyright (c) 2017 Aplix and/or its affiliates. All rights reserved. */

#ifndef ___TIMESLOT_ADVERTISER_H___
#define ___TIMESLOT_ADVERTISER_H___

#include "ble.h"
#include "ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PDU_TYPE_ADV_IND        (0x00)
#define PDU_TYPE_DIRECT_ADV_IND (0x01)
#define PDU_TYPE_NONCONN_IND    (0x02)
#define PDU_TYPE_SCAN_REQ       (0x03)
#define PDU_TYPE_SCAN_RSP       (0x04)
#define PDU_TYPE_CONNECT_REQ    (0x05)
#define PDU_TYPE_SCAN_IND       (0x06)

#define TS_ADV_ADDR_OFFSET      (3)
#define TS_ADV_PACKET_SIZE      (TS_ADV_ADDR_OFFSET + BLE_GAP_ADDR_LEN + BLE_GAP_ADV_SET_DATA_SIZE_MAX)

typedef struct{
    uint8_t packet[TS_ADV_PACKET_SIZE];
} ts_adv_packet_t;

/* 送信パケット指定する関数
   戻り値でレスポンスの有無を返す。

  この関数は最優先の割り込みの中から呼び出されるのことに注意。
 */
typedef bool (*ts_adv_packet_get_func_t)(ts_adv_packet_t * p_adv_packet,
                                         ts_adv_packet_t * p_rsp_packet);

typedef struct{
    ts_adv_packet_get_func_t packet_get_func;
} ts_adv_config_t;

typedef struct{
    uint32_t                interval_us;
    uint32_t                max_delay_us;

    ble_gap_ch_mask_t       channel_mask;

    int8_t                  tx_power; /* RADIO_TXPOWER_TXPOWER_Pos4dBm  or
                                         RADIO_TXPOWER_TXPOWER_Pos3dBm  or
                                         RADIO_TXPOWER_TXPOWER_0dBm     or
                                         RADIO_TXPOWER_TXPOWER_Neg4dBm  or
                                         RADIO_TXPOWER_TXPOWER_Neg8dBm  or
                                         RADIO_TXPOWER_TXPOWER_Neg12dBm or
                                         RADIO_TXPOWER_TXPOWER_Neg16dBm or
                                         RADIO_TXPOWER_TXPOWER_Neg20dBm or
                                         RADIO_TXPOWER_TXPOWER_Neg40dBm */

    uint8_t                 priority; /* NRF_RADIO_PRIORITY_HIGH or
                                         NRF_RADIO_PRIORITY_NORMAL */
} ts_adv_params_t;

typedef struct{
    ble_gap_addr_t  addr;
    uint8_t         pdu_type;
    uint8_t         payload[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    uint8_t         payload_length;
} ts_adv_packet_config_t;

/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t ts_adv_init(ts_adv_config_t const* p_config);

uint32_t ts_adv_params_set(ts_adv_params_t const* p_param);

uint32_t ts_adv_start(void);

uint32_t ts_adv_stop(void);

void ts_adv_packet_setup(ts_adv_packet_t * p_packet,
                         ts_adv_packet_config_t const* p_config);


#ifdef __cplusplus
}
#endif

#endif
