#include "ADCBAT.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ADCBAT";

#define BAT_ADC_GPIO     17
#define BAT_ADC_UNIT     ADC_UNIT_2
#define BAT_ADC_CHANNEL  ADC_CHANNEL_6
#define BAT_ADC_ATTEN    3  // 12dB attenuation

// Voltage divider: 300k + 100k
#define BAT_DIVIDER_NUM  4
#define BAT_DIVIDER_DEN  1

// Li-ion battery voltage range
#define BAT_FULL_MV      4200
#define BAT_EMPTY_MV     3300

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_cali_enabled = false;

void ADCBAT_Init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, BAT_ADC_CHANNEL, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .chan = BAT_ADC_CHANNEL,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle);
    if (ret == ESP_OK) {
        adc_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration init failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Battery ADC initialized on GPIO%d", BAT_ADC_GPIO);
}

uint16_t ADCBAT_ReadVoltageMV(void)
{
    int raw = 0;
    int adc_mv = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, BAT_ADC_CHANNEL, &raw));

    if (adc_cali_enabled) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &adc_mv));
    } else {
        adc_mv = (raw * 1100) / 4095;
    }

    uint32_t bat_mv = (uint32_t)adc_mv * BAT_DIVIDER_NUM / BAT_DIVIDER_DEN;

    ESP_LOGD(TAG, "raw=%d adc=%dmV bat=%dmV", raw, adc_mv, bat_mv);

    return (uint16_t)bat_mv;
}

uint8_t ADCBAT_ReadPercentage(void)
{
    uint16_t bat_mv = ADCBAT_ReadVoltageMV();

    if (bat_mv >= BAT_FULL_MV) return 100;
    if (bat_mv <= BAT_EMPTY_MV) return 0;

    return (uint8_t)((bat_mv - BAT_EMPTY_MV) * 100 / (BAT_FULL_MV - BAT_EMPTY_MV));
}
