#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>
#include <stdbool.h>

// EVQWJN005 trackball + AN48841B-NL hall sensors
// Layout (top view, screen at top):
//   C(G5)  B(G4)
//   D(G6)  A(G3)
#define INPUT_GPIO_A_VO_D  3   // DOWN
#define INPUT_GPIO_B_VO_D  4   // RIGHT
#define INPUT_GPIO_C_VO_D  5   // UP
#define INPUT_GPIO_D_VO_D  6   // LEFT
#define INPUT_GPIO_PUSH    7   // CENTER BUTTON

// Panasonic EVQWJN005 + AN48841B-NL
// If directions/button feel inverted or auto-trigger, toggle this.
#define INPUT_ACTIVE_LOW   1

typedef enum {
    INPUT_NONE = 0,
    INPUT_UP,
    INPUT_DOWN,
    INPUT_LEFT,
    INPUT_RIGHT,
    INPUT_OK,       // short press
    INPUT_LONG_OK,  // long press (>800ms)
} InputEvent;

void Input_Init(void);
InputEvent Input_GetEvent(void);
bool Input_IsPressed(void);
int  Input_GetPushRaw(void);
int  Input_GetDirRaw(uint8_t idx); // 0=A,1=B,2=C,3=D

#endif
