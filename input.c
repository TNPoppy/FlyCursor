#include "input.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#if INPUT_ACTIVE_LOW
#define IS_ACTIVE(level) (!(level))
#else
#define IS_ACTIVE(level) (level)
#endif

static const char *TAG = "INPUT";

// ---------- Trackball: interrupt-based step counters (refer to BlueGo logic) ----------
static volatile int tkb_up_cnt    = 0;
static volatile int tkb_down_cnt  = 0;
static volatile int tkb_left_cnt  = 0;
static volatile int tkb_right_cnt = 0;

static void IRAM_ATTR tkb_isr_up(void *arg)    { tkb_up_cnt++; }
static void IRAM_ATTR tkb_isr_down(void *arg)  { tkb_down_cnt++; }
static void IRAM_ATTR tkb_isr_left(void *arg)  { tkb_left_cnt++; }
static void IRAM_ATTR tkb_isr_right(void *arg) { tkb_right_cnt++; }

void Input_Init(void)
{
    // Configure all pins as input with pull-up (Hall sensors are open-drain, active low)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INPUT_GPIO_A_VO_D)
                      | (1ULL << INPUT_GPIO_B_VO_D)
                      | (1ULL << INPUT_GPIO_C_VO_D)
                      | (1ULL << INPUT_GPIO_D_VO_D)
                      | (1ULL << INPUT_GPIO_PUSH),
        .mode = GPIO_MODE_INPUT,
#if INPUT_ACTIVE_LOW
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#endif
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service (safe to call even if already installed)
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %d", err);
    }

    // Attach ISR handlers for each direction
    // Layout (top view, screen at top):
    //   C(G5)  B(G4)
    //   D(G6)  A(G3)
    gpio_set_intr_type(INPUT_GPIO_C_VO_D, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(INPUT_GPIO_C_VO_D, tkb_isr_up,    NULL);

    gpio_set_intr_type(INPUT_GPIO_A_VO_D, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(INPUT_GPIO_A_VO_D, tkb_isr_down,  NULL);

    gpio_set_intr_type(INPUT_GPIO_D_VO_D, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(INPUT_GPIO_D_VO_D, tkb_isr_left,  NULL);

    gpio_set_intr_type(INPUT_GPIO_B_VO_D, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(INPUT_GPIO_B_VO_D, tkb_isr_right, NULL);
}

InputEvent Input_GetEvent(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // ---------- Direction: read and clear counters ----------
    int up    = tkb_up_cnt;
    int down  = tkb_down_cnt;
    int left  = tkb_left_cnt;
    int right = tkb_right_cnt;

    tkb_up_cnt    = 0;
    tkb_down_cnt  = 0;
    tkb_left_cnt  = 0;
    tkb_right_cnt = 0;

    // Choose dominant direction (same logic as BlueGo get_track_ball_movement_key)
    InputEvent dir_evt = INPUT_NONE;
    if (up > 0 || down > 0 || left > 0 || right > 0) {
        if (up >= down && up >= left && up >= right) {
            dir_evt = INPUT_UP;
        } else if (down >= up && down >= left && down >= right) {
            dir_evt = INPUT_DOWN;
        } else if (left >= up && left >= down && left >= right) {
            dir_evt = INPUT_LEFT;
        } else if (right >= up && right >= down && right >= left) {
            dir_evt = INPUT_RIGHT;
        }
    }

    if (dir_evt != INPUT_NONE) {
        return dir_evt;
    }

    // ---------- Key: shift-register debounce ----------
    // 4-sample history: need 4 consistent reads (4*20ms=80ms) to confirm.
    static uint8_t key_hist = 0;
    static bool    key_confirmed = false;
    static uint32_t key_down_tick = 0;

    bool key = IS_ACTIVE(gpio_get_level(INPUT_GPIO_PUSH));

    key_hist = ((key_hist << 1) | (key ? 1 : 0)) & 0x0F;

    if (!key_confirmed && key_hist == 0x0F) {
        // confirmed press
        key_confirmed = true;
        key_down_tick = now;
    } else if (key_confirmed && key_hist == 0x00) {
        // confirmed release
        key_confirmed = false;
        if (now - key_down_tick < 2000) {
            return INPUT_OK;
        }
    } else if (key_confirmed && (now - key_down_tick >= 2000)) {
        // long press while still held
        key_confirmed = false;
        key_hist = 0x00;
        return INPUT_LONG_OK;
    }

    return INPUT_NONE;
}

bool Input_IsPressed(void)
{
    return IS_ACTIVE(gpio_get_level(INPUT_GPIO_PUSH));
}

int Input_GetPushRaw(void)
{
    return gpio_get_level(INPUT_GPIO_PUSH);
}

int Input_GetDirRaw(uint8_t idx)
{
    switch (idx) {
        case 0: return gpio_get_level(INPUT_GPIO_A_VO_D);
        case 1: return gpio_get_level(INPUT_GPIO_B_VO_D);
        case 2: return gpio_get_level(INPUT_GPIO_C_VO_D);
        case 3: return gpio_get_level(INPUT_GPIO_D_VO_D);
        default: return -1;
    }
}
