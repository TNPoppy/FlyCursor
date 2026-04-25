#ifndef FLYMOUSE_H
#define FLYMOUSE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LSM6DS3/LSM6DSL for fly mouse mode.
 * @param bus_handle I2C master bus handle (shared with other devices).
 * @return ESP_OK on success.
 */
esp_err_t flymouse_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Check if sensor is ready/available.
 */
bool flymouse_is_ready(void);

/**
 * @brief Read gyroscope and calculate mouse movement.
 * @param dx Output X movement (-128 ~ 127).
 * @param dy Output Y movement (-128 ~ 127).
 * @return ESP_OK on success.
 */
esp_err_t flymouse_get_motion(int8_t *dx, int8_t *dy);

/**
 * @brief Read raw sensor data (accel + gyro).
 */
esp_err_t flymouse_read_raw(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                            int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#ifdef __cplusplus
}
#endif

#endif
