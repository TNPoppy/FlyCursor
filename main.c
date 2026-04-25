#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "ssd1306.h"
#include "wouo_ui.h"
#include "input.h"
#include "drv2605.h"
#include "ADCBAT.h"
#include "bluetooth.h"
#include "flymouse.h"
#include "nvs_flash.h"

static const char *TAG = "SSD1306";

#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 0
#define I2C_MASTER_PORT I2C_NUM_0

#define TEST_BUTTON_GPIO 11

static void app_loop_task(void *pvParameters)
{
    i2c_master_bus_handle_t i2c_bus_handle = (i2c_master_bus_handle_t)pvParameters;
    (void)i2c_bus_handle;

    // Test button on GPIO11 (external pull-up, active low)
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << TEST_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    // brief full-white test
    ssd1306_fill_screen(1);
    vTaskDelay(pdMS_TO_TICKS(300));
    ssd1306_clear();

    uint8_t current_page = 0;   // 0 = FlyCursor, 1 = InputTest
    int8_t cursor_x = 31;
    int8_t cursor_y = 15;       // center of 64x32 screen (2x2 dot)

    while (1) {
        static int8_t hid_dx = 0;
        static int8_t hid_dy = 0;
        static uint8_t hid_buttons = 0;

        InputEvent evt = Input_GetEvent();
        if (evt != INPUT_NONE) {
            ESP_LOGI(TAG, "evt=%d page=%d", evt, current_page);
        }

        switch (evt) {
            case INPUT_UP:
                hid_dy -= 8;
                if (current_page == 1 && cursor_y > 0) cursor_y -= 2;
                break;
            case INPUT_DOWN:
                hid_dy += 8;
                if (current_page == 1 && cursor_y < 30) cursor_y += 2;
                break;
            case INPUT_LEFT:
                hid_dx -= 8;
                if (current_page == 1 && cursor_x > 0) cursor_x -= 2;
                break;
            case INPUT_RIGHT:
                hid_dx += 8;
                if (current_page == 1 && cursor_x < 62) cursor_x += 2;
                break;
            case INPUT_OK:
                if (current_page == 0) {
                    current_page = 1;  // FlyCursor -> InputTest
                } else if (current_page == 1) {
                    cursor_x = 31;     // reset cursor to center
                    cursor_y = 15;
                }
                break;
            case INPUT_LONG_OK:
                break;
            default:
                break;
        }

        // ---------- Test button debounce (GPIO11, external pull-up, active low) ----------
        static uint8_t btn_hist = 0;
        static bool btn_confirmed = false;
        static bool btn_is_long = false;
        static uint32_t btn_down_tick = 0;
        static uint32_t btn_last_vibe = 0;
        #define BTN_LONG_PRESS_MS 400
        #define BTN_VIBE_INTERVAL_MS 60

        btn_hist = ((btn_hist << 1) | (gpio_get_level(TEST_BUTTON_GPIO) ? 1 : 0)) & 0x0F;

        if (!btn_confirmed && btn_hist == 0x00) {
            // Just pressed
            btn_confirmed = true;
            btn_is_long = false;
            btn_down_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
            hid_buttons |= 0x01;  // Left button press
        } else if (btn_confirmed && btn_hist != 0x0F) {
            // Still holding
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (!btn_is_long && (now - btn_down_tick >= BTN_LONG_PRESS_MS)) {
                btn_is_long = true;
                btn_last_vibe = now;
                drv2605_play(EFFECT_STRONG_CLICK);
                ESP_LOGI(TAG, "Test button long press start");
            } else if (btn_is_long && (now - btn_last_vibe >= BTN_VIBE_INTERVAL_MS)) {
                btn_last_vibe = now;
                drv2605_play(EFFECT_STRONG_CLICK);
            }
        } else if (btn_confirmed && btn_hist == 0x0F) {
            // Just released
            btn_confirmed = false;
            hid_buttons &= ~0x01; // Left button release
            if (btn_is_long) {
                ESP_LOGI(TAG, "Test button long press stop");
            } else {
                drv2605_play(EFFECT_CLICK);  // Short press: vibrate once
                ESP_LOGI(TAG, "Test button short press");
            }
        }

        // ---------- Battery percentage display (top-right corner) ----------
        static uint8_t bat_pct = 0;
        static uint32_t bat_last_read = 0;
        uint32_t bat_now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (bat_now - bat_last_read >= 1000) {
            bat_last_read = bat_now;
            bat_pct = ADCBAT_ReadPercentage();
        }
        char bat_str[8];
        snprintf(bat_str, sizeof(bat_str), "%d%%", bat_pct);
        uint8_t bat_x = 64 - (strlen(bat_str) * 6);
        if (bat_x > 64) bat_x = 0;

        UI_Clear();

        if (current_page == 0) {
            UI_DrawString(14, 4, "Fly");
            UI_DrawString(10, 14, "Cursor");
        } else {
            UI_DrawRect(0, 0, 64, 32, UI_COLOR_WHITE);
            UI_FillRect(cursor_x, cursor_y, 2, 2, UI_COLOR_WHITE);
        }

        UI_DrawString(bat_x, 0, bat_str);

        // Read gyroscope motion from FlyMouse
        // Set to 0 to disable flymouse and test BLE stability without I2C load
        #define FLYMOUSE_ENABLED 0
        int8_t gyro_dx = 0, gyro_dy = 0;
        #if FLYMOUSE_ENABLED
        if (flymouse_is_ready()) {
            flymouse_get_motion(&gyro_dx, &gyro_dy);
        }
        #endif

        // Send BLE HID mouse report (trackball + gyro merged)
        if (ble_hid_is_connected()) {
            int16_t total_dx = (int16_t)hid_dx + (int16_t)gyro_dx;
            int16_t total_dy = (int16_t)hid_dy + (int16_t)gyro_dy;
            if (total_dx > 127) total_dx = 127;
            if (total_dx < -128) total_dx = -128;
            if (total_dy > 127) total_dy = 127;
            if (total_dy < -128) total_dy = -128;
            ble_hid_send_mouse((int8_t)total_dx, (int8_t)total_dy, 0, hid_buttons);
        }
        hid_dx = 0;
        hid_dy = 0;

        UI_Refresh();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SSD1306 OLED 64x32 wouoUI + Trackball + DRV2605 + BLE HID Starting...");

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C initialized: SCL=GPIO21, SDA=GPIO0");

    ssd1306_init(i2c_bus_handle);
    UI_Init();
    Input_Init();
    drv2605_init(i2c_bus_handle);
    ADCBAT_Init();
    ble_hid_init();
    flymouse_init(i2c_bus_handle);
    ESP_LOGI(TAG, "SSD1306 + wouoUI + Input + DRV2605 + ADCBAT + BLE HID + FlyMouse init done");

    // Run application loop on Core 1, leaving Core 0 for Bluedroid/BLE stack
    xTaskCreatePinnedToCore(app_loop_task, "app_loop", 8192, (void *)i2c_bus_handle, 5, NULL, 1);

    // Suspend app_main so it does not interfere, but keep the task alive
    vTaskSuspend(NULL);
}
