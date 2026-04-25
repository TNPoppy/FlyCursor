idf_component_register(SRCS "flymouse.c" "bluetooth.c" "ADCBAT.c" "input.c" "wouo_ui.c" "main.c" "ssd1306.c" "drv2605.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash esp_driver_i2c esp_driver_gpio esp_adc esp_hid)