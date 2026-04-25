#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// HID Report IDs
#define HID_REPORT_MOUSE_ID         1
#define HID_REPORT_KEYBOARD_ID      2
#define HID_REPORT_CONSUMER_ID      3
#define HID_REPORT_VOICE_ID         4

// HID Report Sizes
#define HID_REPORT_SIZE_MOUSE       5
#define HID_REPORT_SIZE_KEYBOARD    8
#define HID_REPORT_SIZE_CONSUMER    4
#define HID_REPORT_SIZE_VOICE       64

// BLE Device Info
#define BLE_DEVICE_NAME             "FlyCursorT"
#define BLE_MANUFACTURER            "VibeCoding"

// Mouse Report (5 bytes) - for future expansion
// Byte 0: Buttons (bit0=left, bit1=right, bit2=middle)
// Byte 1: X movement (signed)
// Byte 2: Y movement (signed)
// Byte 3: Wheel (signed)
// Byte 4: Reserved
typedef struct __attribute__((packed)) {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
    uint8_t reserved;
} hid_mouse_report_t;

// Keyboard Report (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t modifier;
    uint8_t reserved;
    uint8_t keycodes[5];
    uint8_t reserved2;
} hid_keyboard_report_t;

// Consumer Control Report (4 bytes)
typedef struct __attribute__((packed)) {
    uint16_t usage_page;
    uint16_t usage;
} hid_consumer_report_t;

// Connection state callback
typedef void (*ble_connection_callback_t)(bool connected);

esp_err_t ble_hid_init(void);

esp_err_t ble_hid_send_mouse(int8_t dx, int8_t dy, int8_t wheel, uint8_t buttons);

esp_err_t ble_hid_send_keyboard(uint8_t modifier, uint8_t *keycodes, uint8_t keycount);

esp_err_t ble_hid_send_consumer(uint16_t usage);

esp_err_t ble_hid_send_voice(uint8_t *data, uint16_t len);

bool ble_hid_is_connected(void);

void ble_hid_register_callback(ble_connection_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_H
