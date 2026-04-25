/*
 * FlyCursor - Bluetooth LE HID Implementation (Dual-Core)
 * BLE HID Mouse on Core 0, App loop on Core 1
 * ESP-IDF v6.0, ESP32-S3 (BLE only, no Classic BR/EDR)
 */

#include "bluetooth.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <stdlib.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_hidd.h"
#include "esp_hidd_transport.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "drv2605.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "BLE_HID";

/* ---------- Forward declarations ---------- */
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* ---------- Advertising params ---------- */
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min        = 0x0020,
    .adv_int_max        = 0x0030,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ---------- HID Report Map (Mouse) ---------- */
static const unsigned char s_flycursor_report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Button)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x03,       //     Usage Maximum (3)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data,Var,Abs)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x03,       //     Input (Const,Var,Abs)
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x06,       //     Input (Data,Var,Rel)
    0xC0,             //   End Collection
    0xC0,             // End Collection
};

/* 16-bit UUID for HID service (0x1812) to fit 31-byte adv payload */
static uint8_t s_hid_service_uuid16[] = { 0x12, 0x18 };

/* ---------- HID Device Config ---------- */
static esp_hid_device_config_t s_hid_config = {
    .vendor_id          = 0x1209,
    .product_id         = 0xA101,
    .version            = 0x0100,
    .device_name        = BLE_DEVICE_NAME,
    .manufacturer_name  = BLE_MANUFACTURER,
    .serial_number      = "0001",
    .report_maps        = NULL,
    .report_maps_len    = 0,
};

static esp_hid_raw_report_map_t s_ble_report_maps[1];
static char s_device_name_with_mac[32];

static esp_hidd_dev_t *s_hid_dev = NULL;
static ble_connection_callback_t s_connection_callback = NULL;
static esp_bd_addr_t s_remote_bda = {0};

/* ---------- Advertising start synchronization ---------- */
static volatile bool s_hid_started = false;
static volatile bool s_adv_data_ready = false;

static void try_start_advertising(void)
{
    if (s_hid_started && s_adv_data_ready) {
        esp_err_t ret = esp_ble_gap_start_advertising(&s_adv_params);
        if (ret) {
            ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Advertising requested");
        }
    }
}

/* ---------- Cross-core state protection ---------- */
static volatile bool s_ble_connected = false;
static volatile bool s_conn_ready = false;
static volatile uint32_t s_consecutive_failures = 0;
static portMUX_TYPE s_conn_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ---------- Inter-core queue for HID reports ---------- */
typedef struct {
    int8_t dx;
    int8_t dy;
    int8_t wheel;
    uint8_t buttons;
} hid_mouse_msg_t;

static QueueHandle_t s_hid_queue = NULL;
static TaskHandle_t s_bt_task_handle = NULL;

#define BT_TASK_STACK_SIZE  8192
#define BT_TASK_PRIORITY    5
#define HID_QUEUE_LEN       16

/* ---------- Spinlock helpers ---------- */
static inline void set_connected(bool connected)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    s_ble_connected = connected;
    if (!connected) {
        s_conn_ready = false;
        s_consecutive_failures = 0;
    }
    portEXIT_CRITICAL(&s_conn_spinlock);
}

static inline bool get_connected(void)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    bool v = s_ble_connected;
    portEXIT_CRITICAL(&s_conn_spinlock);
    return v;
}

static inline bool get_conn_ready(void)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    bool v = s_conn_ready;
    portEXIT_CRITICAL(&s_conn_spinlock);
    return v;
}

static inline void set_conn_ready(bool ready)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    s_conn_ready = ready;
    portEXIT_CRITICAL(&s_conn_spinlock);
}

static inline void inc_failure(void)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    s_consecutive_failures++;
    portEXIT_CRITICAL(&s_conn_spinlock);
}

static inline uint32_t get_failures(void)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    uint32_t v = s_consecutive_failures;
    portEXIT_CRITICAL(&s_conn_spinlock);
    return v;
}

static inline void reset_failures(void)
{
    portENTER_CRITICAL(&s_conn_spinlock);
    s_consecutive_failures = 0;
    portEXIT_CRITICAL(&s_conn_spinlock);
}

/* ---------- Core 0 BLE Task: consumes HID queue ---------- */
static void bt_task(void *pvParameters)
{
    (void)pvParameters;
    hid_mouse_msg_t msg;

    ESP_LOGI(TAG, "BT task started on Core %d", xPortGetCoreID());

    for (;;) {
        if (xQueueReceive(s_hid_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_hid_dev) {
            continue;
        }

        if (!get_connected()) {
            continue;
        }

        uint8_t report[3] = {
            msg.buttons & 0x07,
            (uint8_t)msg.dx,
            (uint8_t)msg.dy,
        };

        esp_err_t ret = esp_hidd_dev_input_set(s_hid_dev, 0, 0, report, sizeof(report));
        if (ret != ESP_OK) {
            if (get_conn_ready()) {
                inc_failure();
                uint32_t failures = get_failures();
                if (failures <= 5) {
                    ESP_LOGD(TAG, "HID report failed: %s (failures=%lu)",
                             esp_err_to_name(ret), failures);
                }
                if (failures > 20) {
                    ESP_LOGW(TAG, "Too many report failures, triggering disconnect...");
                    esp_bd_addr_t zero_addr = {0};
                    if (memcmp(s_remote_bda, zero_addr, sizeof(esp_bd_addr_t)) != 0) {
                        esp_ble_gap_disconnect(s_remote_bda);
                    }
                }
            }
        } else {
            reset_failures();
            set_conn_ready(true);
        }
    }
}

/* ---------- GAP Event Handler (runs on Core 0) ---------- */
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data set complete");
        s_adv_data_ready = true;
        try_start_advertising();
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE advertising started");
        } else {
            ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE advertising stopped");
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "BLE security request from peer");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "BLE auth %s with " ESP_BD_ADDR_STR,
                 param->ble_security.auth_cmpl.success ? "success" : "failed",
                 ESP_BD_ADDR_HEX(bd_addr));
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(TAG, "Pairing failed, reason: 0x%x", param->ble_security.auth_cmpl.fail_reason);
            /* Do NOT disconnect here; let the host decide whether to retry or fall back */
        }
        break;
    }

    default:
        break;
    }
}

/* ---------- GATTS Event Handler (runs on Core 0) ---------- */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_CONNECT_EVT:
        memcpy(s_remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, "GATTS disconnect, reason: 0x%04X", param->disconnect.reason);
        memset(s_remote_bda, 0, sizeof(esp_bd_addr_t));
        break;
    default:
        break;
    }
    esp_hidd_gatts_event_handler(event, gatts_if, param);
}

/* ---------- HID Device Event Callback (runs on Core 0) ---------- */
static void ble_hid_event_callback(void *event_handler_arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    (void)event_handler_arg;
    (void)event_base;

    esp_hidd_event_t event = (esp_hidd_event_t)event_id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "BLE HID device started");
        drv2605_play(EFFECT_3_BUZZ);
        s_hid_started = true;
        try_start_advertising();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "BLE HID connected");
        set_connected(true);
        set_conn_ready(false);
        reset_failures();
        drv2605_play(EFFECT_2_BUZZ);
        if (s_connection_callback) {
            s_connection_callback(true);
        }
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "BLE HID disconnected (failures=%lu, ready=%d)",
                 get_failures(), get_conn_ready());
        set_connected(false);
        memset(s_remote_bda, 0, sizeof(esp_bd_addr_t));
        /* Adv data already configured; restart directly */
        esp_ble_gap_start_advertising(&s_adv_params);
        if (s_connection_callback) {
            s_connection_callback(false);
        }
        break;

    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "BLE HID control event: %s",
                 param->control.control ? "exit suspend" : "suspend");
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        ESP_LOGD(TAG, "BLE HID output report received");
        break;

    default:
        break;
    }
}

/* ---------- Public API ---------- */

esp_err_t ble_hid_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing BLE HID device...");

    /* Create inter-core queue and Core-0 task before stack init */
    s_hid_queue = xQueueCreate(HID_QUEUE_LEN, sizeof(hid_mouse_msg_t));
    if (!s_hid_queue) {
        ESP_LOGE(TAG, "Failed to create HID queue");
        return ESP_FAIL;
    }

    BaseType_t task_ret = xTaskCreatePinnedToCore(bt_task, "bt_task",
                                                  BT_TASK_STACK_SIZE, NULL,
                                                  BT_TASK_PRIORITY, &s_bt_task_handle, 0);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BT task on Core 0");
        vQueueDelete(s_hid_queue);
        s_hid_queue = NULL;
        return ESP_FAIL;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Reduce TX power to mitigate power-supply ripple / self-oscillation */
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N6);
    ESP_LOGI(TAG, "BLE TX power set to -6 dBm");

    /* Secure Connections + Bonding, IO_CAP_NONE (Just Works) */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    ret |= esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    ret |= esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    ret |= esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    ret |= esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    if (ret) {
        ESP_LOGE(TAG, "Set security param failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "BLE security: SC+BOND, IO_CAP_NONE (Just Works)");

    ret = esp_ble_gap_register_callback(ble_gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Append public MAC suffix to device name for unique per-board pairing */
    const uint8_t *pub_mac = esp_bt_dev_get_address();
    snprintf(s_device_name_with_mac, sizeof(s_device_name_with_mac),
             "%s-%02X%02X", BLE_DEVICE_NAME, pub_mac[4], pub_mac[5]);
    s_hid_config.device_name = s_device_name_with_mac;

    ret = esp_ble_gap_set_device_name(s_hid_config.device_name);
    if (ret) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Advertising data with 16-bit UUID to fit within 31 bytes */
    static esp_ble_adv_data_t s_adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x03C2,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(s_hid_service_uuid16),
        .p_service_uuid = s_hid_service_uuid16,
        .flag = 0x06,
    };

    ret = esp_ble_gap_config_adv_data(&s_adv_data);
    if (ret) {
        ESP_LOGE(TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_ble_report_maps[0].data = (unsigned char *)s_flycursor_report_map;
    s_ble_report_maps[0].len = sizeof(s_flycursor_report_map);
    s_hid_config.report_maps = s_ble_report_maps;
    s_hid_config.report_maps_len = 1;

    ret = esp_hidd_dev_init(&s_hid_config, ESP_HID_TRANSPORT_BLE, ble_hid_event_callback, &s_hid_dev);
    if (ret) {
        ESP_LOGE(TAG, "HID dev init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BLE HID device initialized");
    ESP_LOGI(TAG, "  Device name: %s", s_hid_config.device_name);
    ESP_LOGI(TAG, "  Public MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             pub_mac[0], pub_mac[1], pub_mac[2], pub_mac[3], pub_mac[4], pub_mac[5]);
    ESP_LOGI(TAG, "  VID: 0x%04X, PID: 0x%04X", s_hid_config.vendor_id, s_hid_config.product_id);

    return ESP_OK;
}

esp_err_t ble_hid_send_mouse(int8_t dx, int8_t dy, int8_t wheel, uint8_t buttons)
{
    (void)wheel;
    if (!s_hid_queue) {
        return ESP_FAIL;
    }

    hid_mouse_msg_t msg = {
        .dx = dx,
        .dy = dy,
        .wheel = wheel,
        .buttons = buttons,
    };

    if (xQueueSend(s_hid_queue, &msg, 0) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_hid_send_keyboard(uint8_t modifier, uint8_t *keycodes, uint8_t keycount)
{
    (void)modifier;
    (void)keycodes;
    (void)keycount;
    return ESP_OK;
}

esp_err_t ble_hid_send_consumer(uint16_t usage)
{
    (void)usage;
    return ESP_OK;
}

bool ble_hid_is_connected(void)
{
    return get_connected();
}

void ble_hid_register_callback(ble_connection_callback_t callback)
{
    s_connection_callback = callback;
}

esp_err_t ble_hid_send_voice(uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    return ESP_OK;
}
