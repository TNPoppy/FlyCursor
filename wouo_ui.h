#ifndef WOUO_UI_H
#define WOUO_UI_H

#include <stdint.h>

#define UI_WIDTH  64
#define UI_HEIGHT 32

typedef enum {
    UI_COLOR_BLACK = 0,
    UI_COLOR_WHITE = 1,
    UI_COLOR_XOR   = 2
} UI_Color;

void UI_Init(void);
void UI_Clear(void);
void UI_SetPixel(uint8_t x, uint8_t y, UI_Color color);
void UI_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, UI_Color color);
void UI_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, UI_Color color);
void UI_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, UI_Color color);
void UI_DrawCircle(uint8_t x, uint8_t y, uint8_t r, UI_Color color);
void UI_DrawString(uint8_t x, uint8_t y, const char *str);
void UI_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap);
void UI_Refresh(void);

#endif
