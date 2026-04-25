/*
 * FlyCursor - DRV2605 LRA/Haptic Driver Implementation
 */

#include "drv2605.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DRV2605";

static bool s_initialized = false;
static bool s_is_lra = true;
static uint32_t s_last_play_time = 0;
#define DRV2605_MIN_PLAY_INTERVAL_MS  50  // Min time between haptic triggers

static i2c_master_dev_handle_t drv2605_dev_handle = NULL;

// ============================================
// I2C Access (New ESP-IDF 5.x Master API)
// ============================================

static esp_err_t drv2605_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(drv2605_dev_handle, buf, 2, -1);
}

static esp_err_t drv2605_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(drv2605_dev_handle, &reg, 1, data, len, -1);
}

// ============================================
// Initialization
// ============================================

esp_err_t drv2605_init(i2c_master_bus_handle_t i2c_bus)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing DRV2605 haptic driver...");

    // Configure EN pin
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << DRV2605_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_conf);
    gpio_set_level(DRV2605_EN_GPIO, 1);  // Enable DRV2605
    vTaskDelay(pdMS_TO_TICKS(5));        // Wait for device to wake up

    // Add DRV2605 to I2C bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DRV2605_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &drv2605_dev_handle));

    // Check status register (should be in standby)
    uint8_t status;
    esp_err_t ret = drv2605_read_reg(DRV2605_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register");
        return ret;
    }

    ESP_LOGI(TAG, "DRV2605 Status: 0x%02X", status);

    // Go to standby mode
    ret = drv2605_write_reg(DRV2605_REG_MODE, DRV2605_MODE_STANDBY);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DRV2605 write failed (QEMU/hardware issue?)");
        // Non-fatal - continue anyway
    }

    // Configure for LRA by default
    ret = drv2605_config_lra();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DRV2605 LRA config failed");
        // Non-fatal - continue anyway
    }

    s_initialized = true;

    ESP_LOGI(TAG, "DRV2605 initialized");

    return ESP_OK;
}

esp_err_t drv2605_enable(bool enable)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(DRV2605_EN_GPIO, enable ? 1 : 0);

    uint8_t mode = enable ? DRV2605_MODE_READY : DRV2605_MODE_STANDBY;
    return drv2605_set_mode(mode);
}

// ============================================
// Configuration
// ============================================

esp_err_t drv2605_set_actuator(bool is_lra)
{
    s_is_lra = is_lra;

    if (is_lra) {
        return drv2605_config_lra();
    } else {
        return drv2605_config_erm();
    }
}

esp_err_t drv2605_config_lra(void)
{
    ESP_LOGI(TAG, "Configuring for LRA actuator...");
    esp_err_t err;

    // Library: LRA Closed Loop
    err = drv2605_write_reg(DRV2605_REG_LIB_SEL, DRV2605_LIB_LRA_ERM_OD);
    if (err != ESP_OK) return err;

    // Feedback: LRA mode, medium drive, BEMF gain 3.3x
    err = drv2605_write_reg(DRV2605_REG_FEEDBACK, 0xB6);
    if (err != ESP_OK) return err;

    // Control1: 4ms overdrive, 50ms brake fade
    err = drv2605_write_reg(DRV2605_REG_CONTROL1, 0x93);
    if (err != ESP_OK) return err;

    // Control2: LRA, 2ms drive, 12.5ms sample time
    err = drv2605_write_reg(DRV2605_REG_CONTROL3, 0x85);
    if (err != ESP_OK) return err;

    // Control4: Auto-resonance, LRA mode
    err = drv2605_write_reg(DRV2605_REG_CONTROL4, 0x2D);
    if (err != ESP_OK) return err;

    // Rated voltage (max 5.0V for strong vibration)
    err = drv2605_write_reg(DRV2605_REG_RATED_V, 0x6E);
    if (err != ESP_OK) return err;

    // Overdrive clamp (max 5.5V for LRA)
    err = drv2605_write_reg(DRV2605_REG_OVERDRIVE_CLAMP, 0xFF);
    if (err != ESP_OK) return err;

    // Calibration compensation (will be updated after calibration)
    err = drv2605_write_reg(DRV2605_REG_CAL_COMP, 0x40);
    if (err != ESP_OK) return err;
    err = drv2605_write_reg(DRV2605_REG_CAL_BEMF, 0x40);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "LRA configuration complete");

    return ESP_OK;
}

esp_err_t drv2605_config_erm(void)
{
    ESP_LOGI(TAG, "Configuring for ERM actuator...");
    esp_err_t err;

    // Library: ERM Closed Loop
    err = drv2605_write_reg(DRV2605_REG_LIB_SEL, DRV2605_LIB_ERM_OD);
    if (err != ESP_OK) return err;

    // Feedback: ERM mode, medium drive, BEMF gain 3.3x
    err = drv2605_write_reg(DRV2605_REG_FEEDBACK, 0x36);
    if (err != ESP_OK) return err;

    // Control1: Default
    err = drv2605_write_reg(DRV2605_REG_CONTROL1, 0x93);
    if (err != ESP_OK) return err;

    // Rated voltage
    err = drv2605_write_reg(DRV2605_REG_RATED_V, 0x5E);
    if (err != ESP_OK) return err;

    // Overdrive clamp
    err = drv2605_write_reg(DRV2605_REG_OVERDRIVE_CLAMP, 0x8C);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "ERM configuration complete");

    return ESP_OK;
}

esp_err_t drv2605_set_library(uint8_t lib_id)
{
    return drv2605_write_reg(DRV2605_REG_LIB_SEL, lib_id);
}

// ============================================
// Playback Control
// ============================================

esp_err_t drv2605_set_mode(uint8_t mode)
{
    return drv2605_write_reg(DRV2605_REG_MODE, mode);
}

esp_err_t drv2605_go(void)
{
    return drv2605_write_reg(DRV2605_REG_GO, 0x01);
}

esp_err_t drv2605_play(uint8_t effect_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Rate limiting - prevent too frequent triggers
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - s_last_play_time < DRV2605_MIN_PLAY_INTERVAL_MS) {
        return ESP_OK;  // Skip this trigger
    }
    s_last_play_time = now;

    // Set to ready mode
    esp_err_t err = drv2605_set_mode(DRV2605_MODE_READY);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "drv2605_set_mode ready failed: %s", esp_err_to_name(err));
        return err;
    }

    // Load effect into sequence
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ1, effect_id);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "drv2605_write_reg WAV_SEQ1 failed: %s", esp_err_to_name(err));
        return err;
    }

    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ2, 0x00);  // End sequence
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "drv2605_write_reg WAV_SEQ2 failed: %s", esp_err_to_name(err));
        return err;
    }

    // Trigger playback
    err = drv2605_go();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "drv2605_go failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "Playing effect: 0x%02X", effect_id);

    return ESP_OK;
}

esp_err_t drv2605_play_strong_long(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Rate limiting
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - s_last_play_time < DRV2605_MIN_PLAY_INTERVAL_MS) {
        return ESP_OK;
    }
    s_last_play_time = now;

    // Set to ready mode
    esp_err_t err = drv2605_set_mode(DRV2605_MODE_READY);
    if (err != ESP_OK) return err;

    // Use ramp up effect (0x11) for smooth buildup then sustained vibration
    // Ramp up + strong buzz for 2 seconds total
    uint8_t effects[] = {
        0x11,  // RAMP_UP - ramp up vibration
        0x06,  // 3_BUZZ - sustained strong vibration
        0x06,  // 3_BUZZ - extend duration
        0x06,  // 3_BUZZ - extend duration
        0x06,  // 3_BUZZ - extend duration (~2 seconds total)
        0x00, 0x00, 0x00, 0x00  // pad rest
    };

    // Load sequence
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ1, effects[0]);
    if (err != ESP_OK) return err;
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ2, effects[1]);
    if (err != ESP_OK) return err;
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ3, effects[2]);
    if (err != ESP_OK) return err;
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ4, effects[3]);
    if (err != ESP_OK) return err;
    err = drv2605_write_reg(DRV2605_REG_WAV_SEQ5, effects[4]);
    if (err != ESP_OK) return err;

    // Trigger playback
    return drv2605_go();
}

esp_err_t drv2605_play_sequence(uint8_t *effects, size_t count)
{
    if (!s_initialized || !effects || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (count > 8) {
        count = 8;  // Max 8 effects in sequence
    }

    // Set to ready mode
    ESP_ERROR_CHECK(drv2605_set_mode(DRV2605_MODE_READY));

    // Load sequence
    uint8_t seq_reg = DRV2605_REG_WAV_SEQ1;
    for (size_t i = 0; i < count; i++) {
        ESP_ERROR_CHECK(drv2605_write_reg(seq_reg++, effects[i]));
    }

    // Clear remaining sequence registers
    for (size_t i = count; i < 8; i++) {
        ESP_ERROR_CHECK(drv2605_write_reg(seq_reg++, 0x00));
    }

    // Trigger playback
    ESP_ERROR_CHECK(drv2605_go());

    return ESP_OK;
}

esp_err_t drv2605_stop(void)
{
    // Write stop command
    return drv2605_write_reg(DRV2605_REG_GO, 0x00);
}

esp_err_t drv2605_pulse(int8_t amplitude)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set to realtime mode
    ESP_ERROR_CHECK(drv2605_set_mode(DRV2605_MODE_REALTIME));

    // Set amplitude (signed 8-bit: -128 to +127)
    ESP_ERROR_CHECK(drv2605_write_reg(DRV2605_REG_RT_INPUT, (uint8_t)amplitude));

    return ESP_OK;
}

// ============================================
// Calibration
// ============================================

esp_err_t drv2605_calibrate(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting calibration...");

    // Set to diagnostic mode
    ESP_ERROR_CHECK(drv2605_set_mode(DRV2605_MODE_DIAGNOSTIC));

    // Trigger calibration
    ESP_ERROR_CHECK(drv2605_go());

    // Wait for completion (check status)
    uint8_t status;
    uint8_t retry = 50;  // 5 seconds max

    while (retry-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_ERROR_CHECK(drv2605_read_reg(DRV2605_REG_STATUS, &status, 1));

        // Check if diagnostic complete
        if (status & 0x08) {
            ESP_LOGI(TAG, "Calibration complete, status: 0x%02X", status);

            // Read calibration values
            uint8_t cal_comp, cal_bemf, lra_period;
            ESP_ERROR_CHECK(drv2605_read_reg(DRV2605_REG_CAL_COMP, &cal_comp, 1));
            ESP_ERROR_CHECK(drv2605_read_reg(DRV2605_REG_CAL_BEMF, &cal_bemf, 1));
            ESP_ERROR_CHECK(drv2605_read_reg(DRV2605_REG_LRA_PERIOD, &lra_period, 1));

            ESP_LOGI(TAG, "Calibration values:");
            ESP_LOGI(TAG, "  Comp: 0x%02X, BEMF: 0x%02X, Period: 0x%02X",
                     cal_comp, cal_bemf, lra_period);

            // Back to standby
            ESP_ERROR_CHECK(drv2605_set_mode(DRV2605_MODE_STANDBY));

            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "Calibration timed out");

    return ESP_ERR_TIMEOUT;
}

// ============================================
// Self Test
// ============================================

esp_err_t drv2605_self_test(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Running self-test...");

    // Play a simple effect
    ESP_ERROR_CHECK(drv2605_play(EFFECT_STRONG_CLICK));

    // Wait for effect to complete
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Self-test complete");

    return ESP_OK;
}
