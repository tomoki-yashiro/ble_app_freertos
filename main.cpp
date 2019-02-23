#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define DEVICE_NAME                         "Nordic_HRM"
#define MANUFACTURER_NAME                   "NordicSemiconductor"

#define APP_BLE_OBSERVER_PRIO               3
#define APP_BLE_CONN_CFG_TAG                1

#define BATTERY_LEVEL_MEAS_INTERVAL         2000
#define MIN_BATTERY_LEVEL                   81
#define MAX_BATTERY_LEVEL                   100
#define BATTERY_LEVEL_INCREMENT             1

#define HEART_RATE_MEAS_INTERVAL            1000
#define MIN_HEART_RATE                      140
#define MAX_HEART_RATE                      300
#define HEART_RATE_INCREMENT                10

#define RR_INTERVAL_INTERVAL                300
#define MIN_RR_INTERVAL                     100
#define MAX_RR_INTERVAL                     500
#define RR_INTERVAL_INCREMENT               1

#define SENSOR_CONTACT_DETECTED_INTERVAL    5000

#define DEAD_BEEF                           0xDEADBEEF

BLE_BAS_DEF(m_bas);
BLE_HRS_DEF(m_hrs);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;
static bool     m_rr_interval_enabled = true;

static sensorsim_cfg_t   m_battery_sim_cfg;
static sensorsim_state_t m_battery_sim_state;
static sensorsim_cfg_t   m_heart_rate_sim_cfg;
static sensorsim_state_t m_heart_rate_sim_state;
static sensorsim_cfg_t   m_rr_interval_sim_cfg;
static sensorsim_state_t m_rr_interval_sim_state;

static ble_uuid_t m_adv_uuids[] =
{
    {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
};

static TimerHandle_t m_battery_timer;
static TimerHandle_t m_heart_rate_timer;
static TimerHandle_t m_rr_interval_timer;
static TimerHandle_t m_sensor_contact_timer;

static void advertising_start(void * p_erase_bonds);

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch(p_evt->evt_id){
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        advertising_start(&delete_bonds);
        break;

    default:
        break;
    }
}

static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state,
                                               &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level,
                                            BLE_CONN_HANDLE_ALL);

    if((err_code != NRF_SUCCESS)             &&
       (err_code != NRF_ERROR_INVALID_STATE) &&
       (err_code != NRF_ERROR_RESOURCES)     &&
       (err_code != NRF_ERROR_BUSY)          &&
       (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)){
        APP_ERROR_HANDLER(err_code);
    }
}

static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}

static void heart_rate_meas_timeout_handler(TimerHandle_t xTimer)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(xTimer);

    heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state,
                                             &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if((err_code != NRF_SUCCESS)             &&
       (err_code != NRF_ERROR_INVALID_STATE) &&
       (err_code != NRF_ERROR_RESOURCES)     &&
       (err_code != NRF_ERROR_BUSY)          &&
       (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)){
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate
    // measurement.  NOTE: An application will normally not do
    // this. It is done here just for testing generation of messages
    // without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}

static void rr_interval_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);

    if(m_rr_interval_enabled){

        uint16_t rr_interval =
            (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                        &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}

static void sensor_contact_detected_timeout_handler(TimerHandle_t xTimer)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(xTimer);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}

static void timers_init(void)
{
    m_battery_timer =
        xTimerCreate("BATT",
                     BATTERY_LEVEL_MEAS_INTERVAL,
                     pdTRUE,
                     NULL,
                     battery_level_meas_timeout_handler);
    m_heart_rate_timer =
        xTimerCreate("HRT",
                     HEART_RATE_MEAS_INTERVAL,
                     pdTRUE,
                     NULL,
                     heart_rate_meas_timeout_handler);
    m_rr_interval_timer =
        xTimerCreate("RRT",
                     RR_INTERVAL_INTERVAL,
                     pdTRUE,
                     NULL,
                     rr_interval_timeout_handler);
    m_sensor_contact_timer =
        xTimerCreate("SCT",
                     SENSOR_CONTACT_DETECTED_INTERVAL,
                     pdTRUE,
                     NULL,
                     sensor_contact_detected_timeout_handler);

    if((NULL == m_battery_timer)     ||
       (NULL == m_heart_rate_timer)  ||
       (NULL == m_rr_interval_timer) ||
       (NULL == m_sensor_contact_timer)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void gap_params_init(void)
{
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    APP_ERROR_CHECK(sd_ble_gap_device_name_set(&sec_mode,
                                               (const uint8_t *)DEVICE_NAME,
                                               strlen(DEVICE_NAME)));

    APP_ERROR_CHECK(sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT));

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);

    APP_ERROR_CHECK(sd_ble_gap_ppcp_set(&gap_conn_params));
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
    // Initialize Queued Write Module.
    nrf_ble_qwr_init_t qwr_init = {0};
    qwr_init.error_handler = nrf_qwr_error_handler;
    APP_ERROR_CHECK(nrf_ble_qwr_init(&m_qwr, &qwr_init));

    // Heart Rate Service.
    uint8_t body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;
    ble_hrs_init_t hrs_init;

    memset(&hrs_init, 0, sizeof(hrs_init));
    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;
    hrs_init.hrm_cccd_wr_sec             = SEC_OPEN;
    hrs_init.bsl_rd_sec                  = SEC_OPEN;
    APP_ERROR_CHECK(ble_hrs_init(&m_hrs, &hrs_init));

    // Battery Service.
    ble_bas_init_t bas_init;

    memset(&bas_init, 0, sizeof(bas_init));
    bas_init.bl_rd_sec            = SEC_OPEN;
    bas_init.bl_cccd_wr_sec       = SEC_OPEN;
    bas_init.bl_report_rd_sec     = SEC_OPEN;
    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    APP_ERROR_CHECK(ble_bas_init(&m_bas, &bas_init));

    // Device Information Service
    ble_dis_init_t dis_init;

    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,
                          (char *)MANUFACTURER_NAME);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    APP_ERROR_CHECK(ble_dis_init(&dis_init));
}

static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;
    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;
    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;
    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}

static void application_timers_start(void)
{
    // Start application timers.
    if(pdPASS != xTimerStart(m_battery_timer, portMAX_DELAY)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if(pdPASS != xTimerStart(m_heart_rate_timer, portMAX_DELAY)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if(pdPASS != xTimerStart(m_rr_interval_timer, portMAX_DELAY)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if(pdPASS != xTimerStart(m_sensor_contact_timer, portMAX_DELAY)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED){
        APP_ERROR_CHECK(sd_ble_gap_disconnect(m_conn_handle,
                                              BLE_HCI_CONN_INTERVAL_UNACCEPTABLE));
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000);
    cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000);
    cp_init.max_conn_params_update_count   = 3;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    APP_ERROR_CHECK(ble_conn_params_init(&cp_init));
}

static void sleep_mode_enter(void)
{
    APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_IDLE));
    APP_ERROR_CHECK(bsp_btn_ble_sleep_mode_prepare());
    APP_ERROR_CHECK(sd_power_system_off());
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch(ble_adv_evt){
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_ADVERTISING));
        break;

    case BLE_ADV_EVT_SLOW:
        NRF_LOG_INFO("Slow advertising.");
        break;

    case BLE_ADV_EVT_IDLE:
        //sleep_mode_enter();
        break;

    default:
        break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    switch(p_ble_evt->header.evt_id){
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        APP_ERROR_CHECK(bsp_indication_set(BSP_INDICATE_CONNECTED));
        APP_ERROR_CHECK(nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle));
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                BLE_GAP_PHY_AUTO,
                BLE_GAP_PHY_AUTO,
            };
        APP_ERROR_CHECK(sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys));
        break;
    }

    case BLE_GATTC_EVT_TIMEOUT:
        NRF_LOG_DEBUG("GATT Client Timeout.");
        APP_ERROR_CHECK(sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        NRF_LOG_DEBUG("GATT Server Timeout.");
        APP_ERROR_CHECK(sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION));
        break;

    default:
        break;
    }
}

static void ble_stack_init(void)
{
    uint32_t ram_start = 0;

    APP_ERROR_CHECK(nrf_sdh_enable_request());
    APP_ERROR_CHECK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG,
                                                &ram_start));
    APP_ERROR_CHECK(nrf_sdh_ble_enable(&ram_start));

    NRF_SDH_BLE_OBSERVER(m_ble_observer,
                         APP_BLE_OBSERVER_PRIO,
                         ble_evt_handler,
                         NULL);
}

static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch(event){
    case BSP_EVENT_SLEEP:
        sleep_mode_enter();
        break;

    case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if(err_code != NRF_ERROR_INVALID_STATE){
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BSP_EVENT_WHITELIST_OFF:
        if(m_conn_handle == BLE_CONN_HANDLE_INVALID){
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            if(err_code != NRF_ERROR_INVALID_STATE){
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

    default:
        break;
    }
}

static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    sec_param.bond           = 1;
    sec_param.mitm           = 0;
    sec_param.lesc           = 0;
    sec_param.keypress       = 0;
    sec_param.io_caps        = BLE_GAP_IO_CAPS_NONE;
    sec_param.oob            = 0;
    sec_param.min_key_size   = 7;
    sec_param.max_key_size   = 16;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    APP_ERROR_CHECK(pm_init());
    APP_ERROR_CHECK(pm_sec_params_set(&sec_param));
    APP_ERROR_CHECK(pm_register(pm_evt_handler));
}

static void delete_bonds(void)
{
    NRF_LOG_INFO("Erase bonds!");
    APP_ERROR_CHECK(pm_peers_delete());
}

static void advertising_init(void)
{
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = MSEC_TO_UNITS(20, UNIT_0_625_MS);
    init.config.ble_adv_fast_timeout  = MSEC_TO_UNITS(30000, UNIT_10_MS);

    init.config.ble_adv_slow_enabled  = true;
    init.config.ble_adv_slow_interval = MSEC_TO_UNITS(417.5, UNIT_0_625_MS);
    init.config.ble_adv_slow_timeout  = 0;

    init.evt_handler = on_adv_evt;

    APP_ERROR_CHECK(ble_advertising_init(&m_advertising, &init));

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    APP_ERROR_CHECK(bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS,
                             bsp_event_handler));
    APP_ERROR_CHECK(bsp_btn_ble_init(NULL, &startup_event));

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if(erase_bonds){
        delete_bonds();
    }else{
        APP_ERROR_CHECK( ble_advertising_start(&m_advertising,
                                               BLE_ADV_MODE_FAST));
    }
}

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;

static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while(1){
        NRF_LOG_FLUSH();
        vTaskSuspend(NULL);
    }
}
#endif //NRF_LOG_ENABLED

int main(void)
{
    bool erase_bonds;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    APP_ERROR_CHECK(nrf_drv_clock_init());

#if NRF_LOG_ENABLED
    if(pdPASS != xTaskCreate(logger_thread,
                             "LOGGER",
                             256,
                             NULL,
                             1,
                             &m_logger_thread)){
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // Initialize modules.
    timers_init();
    buttons_leds_init(&erase_bonds);
    gap_params_init();
    APP_ERROR_CHECK(nrf_ble_gatt_init(&m_gatt, NULL));
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
    application_timers_start();

    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    NRF_LOG_INFO("Application started.");
    vTaskStartScheduler();

    for(;;){
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

extern "C"
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

extern "C"
void vApplicationIdleHook(void)
{
#if NRF_LOG_ENABLED
    vTaskResume(m_logger_thread);
#endif
}

