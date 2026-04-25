#ifndef SSD1306_H
#define SSD1306_H

#include "driver/i2c_master.h"

void ssd1306_init(i2c_master_bus_handle_t i2c_bus_handle);
void ssd1306_clear(void);
void ssd1306_clear_line(uint8_t page);
void ssd1306_set_cursor(uint8_t col, uint8_t page);
void ssd1306_write_char(uint8_t ch);
void ssd1306_write_string(const char *str);
void ssd1306_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bitmap);
void ssd1306_fill_screen(uint8_t fill);
void ssd1306_test_pattern(void);
void ssd1306_send_data(const uint8_t *data, size_t len);

#endif
