#include "flymouse.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FLYMOUSE";

// LSM6DS3/LSM6DSL registers
#define REG_WHO_AM_I       0x0F
#define REG_CTRL1_XL       0x10
#define REG_CTRL2_G        0x11
#define REG_CTRL3_C        0x12
#define REG_STATUS         0x1E
#define REG_OUTX_L_G       0x22

#define WHO_AM_I_LSM6DS3   0x69
#define WHO_AM_I_LSM6DSL   0x6A

#define I2C_ADDR           0x6A
#define SENSITIVITY_DIV    64
#define CALIB_SAMPLES      64
#define GYRO_DEADZONE      8   // raw units, ~0.5 DPS at 2000DPS scale

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_ready = false;

// Gyro zero-rate bias (calibrated at init)
static int16_t s_gyro_bias_x = 0;
static int16_t s_gyro_bias_y = 0;
static int16_t s_gyro_bias_z = 0;

// I2C fault tolerance
static uint8_t s_i2c_fail_count = 0;
#define I2C_FAIL_THRESHOLD  5
#define I2C_TIMEOUT_MS      5   // was 20ms, tighten to avoid blocking BLE task

static esp_err_t read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(s_dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static bool gyro_data_ready(void)
{
    uint8_t status = 0;
    if (read_reg(REG_STATUS, &status, 1) != ESP_OK) return false;
    return (status & 0x02) != 0;  // G_DA (gyro data available) bit
}

esp_err_t flymouse_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    uint8_t who = 0;
    ret = read_reg(REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LSM6DS3 not responding (I2C timeout)");
        return ret;
    }
    if (who != WHO_AM_I_LSM6DS3 && who != WHO_AM_I_LSM6DSL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x69/0x6A, got 0x%02X", who);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "IMU found, WHO_AM_I: 0x%02X", who);
    s_ready = true;

    // Accelerometer: 104Hz ODR, 8G full scale
    ret = write_reg(REG_CTRL1_XL, 0x4C);
    if (ret != ESP_OK) return ret;

    // Gyroscope: 104Hz ODR, 2000DPS full scale
    ret = write_reg(REG_CTRL2_G, 0x4C);
    if (ret != ESP_OK) return ret;

    // BDU + address auto-increment
    ret = write_reg(REG_CTRL3_C, 0x44);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    // Gyro zero-rate calibration
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        uint8_t buf[6];
        if (read_reg(REG_OUTX_L_G, buf, 6) == ESP_OK) {
            sum_x += (int16_t)(buf[1] << 8 | buf[0]);
            sum_y += (int16_t)(buf[3] << 8 | buf[2]);
            sum_z += (int16_t)(buf[5] << 8 | buf[4]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_gyro_bias_x = (int16_t)(sum_x / CALIB_SAMPLES);
    s_gyro_bias_y = (int16_t)(sum_y / CALIB_SAMPLES);
    s_gyro_bias_z = (int16_t)(sum_z / CALIB_SAMPLES);
    ESP_LOGI(TAG, "Gyro bias calibrated: X=%d Y=%d Z=%d",
             s_gyro_bias_x, s_gyro_bias_y, s_gyro_bias_z);

    ESP_LOGI(TAG, "FlyMouse initialized");
    return ESP_OK;
}

bool flymouse_is_ready(void)
{
    return s_ready;
}

esp_err_t flymouse_read_raw(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                            int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;

    uint8_t buf[12];
    esp_err_t ret = read_reg(REG_OUTX_L_G, buf, 12);
    if (ret != ESP_OK) return ret;

    *gyro_x = (int16_t)(buf[1] << 8 | buf[0]);
    *gyro_y = (int16_t)(buf[3] << 8 | buf[2]);
    *gyro_z = (int16_t)(buf[5] << 8 | buf[4]);
    *acc_x  = (int16_t)(buf[7] << 8 | buf[6]);
    *acc_y  = (int16_t)(buf[9] << 8 | buf[8]);
    *acc_z  = (int16_t)(buf[11] << 8 | buf[10]);

    return ESP_OK;
}

esp_err_t flymouse_get_motion(int8_t *dx, int8_t *dy)
{
    *dx = 0;
    *dy = 0;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!gyro_data_ready()) {
        return ESP_OK;  // no new data, not an error
    }

    uint8_t buf[6];
    esp_err_t ret = read_reg(REG_OUTX_L_G, buf, 6);
    if (ret != ESP_OK) {
        s_i2c_fail_count++;
        if (s_i2c_fail_count >= I2C_FAIL_THRESHOLD) {
            ESP_LOGE(TAG, "I2C failed %d times, marking unavailable", s_i2c_fail_count);
            s_ready = false;
        }
        return ret;
    }
    s_i2c_fail_count = 0;

    int16_t gyro_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t gyro_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gyro_z = (int16_t)(buf[5] << 8 | buf[4]);

    gyro_x -= s_gyro_bias_x;
    gyro_y -= s_gyro_bias_y;
    gyro_z -= s_gyro_bias_z;

    // Apply deadzone to suppress drift when stationary
    if (gyro_z > -GYRO_DEADZONE && gyro_z < GYRO_DEADZONE) gyro_z = 0;
    if (gyro_y > -GYRO_DEADZONE && gyro_y < GYRO_DEADZONE) gyro_y = 0;

    // Map gyro to mouse movement
    // gyro_z (yaw) -> X axis
    // gyro_y (pitch, inverted) -> Y axis
    int16_t raw_dx = gyro_z / SENSITIVITY_DIV;
    int16_t raw_dy = -gyro_y / SENSITIVITY_DIV;

    if (raw_dx > 127) raw_dx = 127;
    if (raw_dx < -128) raw_dx = -128;
    if (raw_dy > 127) raw_dy = 127;
    if (raw_dy < -128) raw_dy = -128;

    *dx = (int8_t)raw_dx;
    *dy = (int8_t)raw_dy;

    return ESP_OK;
}
