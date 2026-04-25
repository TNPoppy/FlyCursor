/*
 * FlyCursor - DRV2605 LRA/Haptic Driver
 * Linear Resonance Actuator (LRA) Driver for Haptic Feedback
 */

#ifndef DRV2605_H
#define DRV2605_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// ============================================
// Pin Configuration
// ============================================
#define DRV2605_EN_GPIO           18

// ============================================
// Device Address
// ============================================
#define DRV2605_ADDR              0x5A

// ============================================
// Registers
// ============================================
#define DRV2605_REG_STATUS         0x00
#define DRV2605_REG_MODE            0x01
#define DRV2605_REG_RT_INPUT       0x02
#define DRV2605_REG_LIB_SEL        0x03
#define DRV2605_REG_WAV_SEQ1       0x04
#define DRV2605_REG_WAV_SEQ2       0x05
#define DRV2605_REG_WAV_SEQ3       0x06
#define DRV2605_REG_WAV_SEQ4       0x07
#define DRV2605_REG_WAV_SEQ5       0x08
#define DRV2605_REG_WAV_SEQ6       0x09
#define DRV2605_REG_WAV_SEQ7       0x0A
#define DRV2605_REG_WAV_SEQ8       0x0B
#define DRV2605_REG_GO             0x0C
#define DRV2605_REG_ODRIVE_SCAL   0x0D
#define DRV2605_REG_SIG_DRIVE_PW   0x0E
#define DRV2605_REG_MID_BRAKE      0x10
#define DRV2605_REG_ODRIVE_TIME    0x11
#define DRV2605_REG_SAMPLE_TIME    0x12
#define DRV2605_REG_FRM_V          0x13
#define DRV2605_REG_BRAKE_TIME     0x14
#define DRV2605_REG_AUDIO_Vibe_CTL 0x15
#define DRV2605_REG_AUDIO_LRA_INPUT 0x16
#define DRV2605_REG_AUDIO_HIPASS_CTL 0x17
#define DRV2605_REG_BIAS           0x18
#define DRV2605_REG_AMPLITUDE      0x19
#define DRV2605_REG_RATED_V        0x1A
#define DRV2605_REG_OVERDRIVE_CLAMP 0x1B
#define DRV2605_REG_NOMINAL_VOLTAGE 0x1C
#define DRV2605_REG_OVERDRIVE_VOLTAGE 0x1D
#define DRV2605_REG_CAL_COMP       0x1E
#define DRV2605_REG_CAL_BEMF       0x1F
#define DRV2605_REG_FEEDBACK        0x20
#define DRV2605_REG_CONTROL1       0x21
#define DRV2605_REG_CONTROL2       0x22
#define DRV2605_REG_CONTROL3       0x23
#define DRV2605_REG_CONTROL4       0x24
#define DRV2605_REG_CONTROL5       0x25
#define DRV2605_REG_LRA_PERIOD     0x26

// ============================================
// Mode Settings
// ============================================
#define DRV2605_MODE_STANDBY       0x00
#define DRV2605_MODE_READY         0x01
#define DRV2605_MODE_INVALID       0x02
#define DRV2605_MODE_DIAGNOSTIC    0x03
#define DRV2605_MODE_AUDIOHAPTIC   0x04
#define DRV2605_MODE_REALTIME      0x05
#define DRV2605_MODE_DC            0x06
#define DRV2605_MODE_TSIP          0x07

// Library Selection
#define DRV2605_LIB_EMPTY          0x00
#define DRV2605_LIB_LRA_ERM       0x01  // LRA/ERM Open Loop
#define DRV2605_LIB_ERM_OD         0x02  // ERM Closed Loop
#define DRV2605_LIB_LRA_ERM_OD     0x03  // LRA Closed Loop
#define DRV2605_LIB_TSIP           0x06  // TSIP

// ============================================
// Preset Effects (ERM Library)
// ============================================
#define EFFECT_NONE                0x00
#define EFFECT_STRONG_CLICK        0x01
#define EFFECT_CLICK               0x02
#define EFFECT_DOUBLE_CLICK        0x03
#define EFFECT_TRIPLE_CLICK        0x04
#define EFFECT_2_BUZZ              0x05
#define EFFECT_3_BUZZ              0x06
#define EFFECT_4_BUZZ              0x07
#define EFFECT_5_BUZZ              0x08
#define EFFECT_RAMP_DOWN           0x10
#define EFFECT_RAMP_UP             0x11
#define EFFECT_RAMP_DOWN_UP        0x12
#define EFFECT_CLICK_STRONG        0x15
#define EFFECT_CLICK_NORMAL        0x16
#define EFFECT_CLICK_SOFT          0x17
#define EFFECT_TRANSITION_CLICK1   0x18
#define EFFECT_TRANSITION_CLICK2   0x19
#define EFFECT_TRANSITION_CLICK3   0x1A
#define EFFECT_TRANSITION_CLICK4  0x1B
#define EFFECT_TRANSITION_CLICK5  0x1C
#define EFFECT_TRANSITION_CLICK6  0x1D
#define EFFECT_TRANSITION_CLICK7  0x1E
#define EFFECT_TRANSITION_CLICK8  0x1F
#define EFFECT_TRANSITION_CLICK9  0x20
#define EFFECT_TRANSITION_CLICK10 0x21
#define EFFECT_TRANSITION_LONG1   0x30
#define EFFECT_TRANSITION_LONG2   0x31

// ============================================
// Function Declarations
// ============================================

// Initialize DRV2605 (pass the shared I2C master bus handle)
esp_err_t drv2605_init(i2c_master_bus_handle_t i2c_bus);

// Set actuator type
esp_err_t drv2605_set_actuator(bool is_lra);  // true = LRA, false = ERM

// Play effect by ID
esp_err_t drv2605_play(uint8_t effect_id);

// Play strong long vibration (~2 seconds, max intensity)
esp_err_t drv2605_play_strong_long(void);

// Play sequence
esp_err_t drv2605_play_sequence(uint8_t *effects, size_t count);

// Stop playback
esp_err_t drv2605_stop(void);

// Trigger single pulse (realtime mode)
esp_err_t drv2605_pulse(int8_t amplitude);  // -128 to +127

// Set library
esp_err_t drv2605_set_library(uint8_t lib_id);

// Set Go bit
esp_err_t drv2605_go(void);

// Set mode
esp_err_t drv2605_set_mode(uint8_t mode);

// Configure for LRA (Linear Resonance Actuator)
esp_err_t drv2605_config_lra(void);

// Configure for ERM (Eccentric Rotating Mass)
esp_err_t drv2605_config_erm(void);

// Calibration
esp_err_t drv2605_calibrate(void);

// Self test
esp_err_t drv2605_self_test(void);

// Enable/disable
esp_err_t drv2605_enable(bool enable);

#endif // DRV2605_H
