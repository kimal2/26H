#include "DisplayTask.h"

#include "OLED.h"
#include "cmsis_os2.h"
#include "linetrack.h"
#include "main.h"

#define MENU_SCAN_PERIOD_MS          10U
#define MENU_REFRESH_PERIOD_MS       100U
#define MENU_DEBOUNCE_SAMPLES        3U

typedef enum {
    MENU_STATE_SELECT = 0,
    MENU_STATE_RUNNING,
    MENU_STATE_COMPLETE
} MenuState_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t stable_samples;
} MenuKey_t;

static MenuKey_t key_up = {KEY_1_GPIO_Port, KEY_1_Pin, 0U, 0U, 0U};
static MenuKey_t key_down = {KEY_2_GPIO_Port, KEY_2_Pin, 0U, 0U, 0U};
static MenuKey_t key_confirm = {KEY_3_GPIO_Port, KEY_3_Pin, 0U, 0U, 0U};
static MenuKey_t key_back = {KEY_4_GPIO_Port, KEY_4_Pin, 0U, 0U, 0U};

static volatile uint8_t tracking_finished;
static MenuState_t menu_state = MENU_STATE_SELECT;
static uint32_t tracking_start_tick;
static uint32_t tracking_finish_tick;
static uint32_t last_refresh_tick;

static uint8_t Menu_KeyRead(const MenuKey_t *key)
{
    return (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void Menu_ForwardGrayKey(void)
{
    GPIO_PinState output_state;

    output_state = (Menu_KeyRead(&key_up) != 0U) ?
                   GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GRAY_KEY_GPIO_Port, GRAY_KEY_Pin, output_state);
}

static void Menu_KeyInit(MenuKey_t *key)
{
    key->raw_pressed = Menu_KeyRead(key);
    key->stable_pressed = key->raw_pressed;
    key->stable_samples = MENU_DEBOUNCE_SAMPLES;
}

static uint8_t Menu_KeyPressed(MenuKey_t *key)
{
    uint8_t pressed = Menu_KeyRead(key);

    if (pressed != key->raw_pressed) {
        key->raw_pressed = pressed;
        key->stable_samples = 1U;
        return 0U;
    }
    if (key->stable_samples < MENU_DEBOUNCE_SAMPLES) {
        key->stable_samples++;
    }
    if ((key->stable_samples >= MENU_DEBOUNCE_SAMPLES) &&
        (key->stable_pressed != pressed)) {
        key->stable_pressed = pressed;
        return pressed;
    }
    return 0U;
}

static void Menu_RenderSelect(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"TASK MENU", 8U);
    OLED_ShowString(0U, 1U, (uint8_t *)"> R2 LINE TRACK", 8U);
    OLED_ShowString(0U, 5U, (uint8_t *)"KEY1: GRAY CAL", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY3: START", 8U);
    OLED_ShowString(0U, 7U, (uint8_t *)"KEY4: STOP", 8U);
}

static void Menu_RenderRunning(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"R2 LINE TRACK", 8U);
    OLED_ShowString(0U, 1U, (uint8_t *)"STATUS: RUNNING", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: ABORT", 8U);
}

static void Menu_RenderResult(const char *status, uint32_t elapsed_ms)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"R2 LINE TRACK", 8U);
    OLED_Printf(0U, 1U, 8U, "STATUS: %s", status);
    OLED_Printf(0U, 3U, 8U, "TIME: %2lu.%1lu S",
                (unsigned long)(elapsed_ms / 1000U),
                (unsigned long)((elapsed_ms % 1000U) / 100U));
    OLED_ShowString(0U, 5U, (uint8_t *)"KEY3: AGAIN", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: MENU", 8U);
}

static void Menu_StartTracking(void)
{
    if (Tracking_IsReady() == 0U) {
        OLED_ShowString(0U, 4U, (uint8_t *)"SENSOR NOT READY", 8U);
        return;
    }

    tracking_finished = 0U;
    tracking_start_tick = HAL_GetTick();
    last_refresh_tick = tracking_start_tick;
    menu_state = MENU_STATE_RUNNING;
    Tracking_Start();
    Menu_RenderRunning();
}

static void Menu_UpdateRunning(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - tracking_start_tick;

    if (tracking_finished != 0U) {
        tracking_finished = 0U;
        tracking_finish_tick = now;
        menu_state = MENU_STATE_COMPLETE;
        Menu_RenderResult("COMPLETE", tracking_finish_tick - tracking_start_tick);
        return;
    }
    if ((now - last_refresh_tick) >= MENU_REFRESH_PERIOD_MS) {
        last_refresh_tick = now;
        OLED_Printf(0U, 3U, 8U, "TIME: %2lu.%1lu S",
                    (unsigned long)(elapsed / 1000U),
                    (unsigned long)((elapsed % 1000U) / 100U));
    }
}

void StartDisplayTask(void *argument)
{
    (void)argument;

    Menu_KeyInit(&key_up);
    Menu_KeyInit(&key_down);
    Menu_KeyInit(&key_confirm);
    Menu_KeyInit(&key_back);
    Menu_RenderSelect();

    for (;;) {
        DisplayTask_Process();
        osDelay(MENU_SCAN_PERIOD_MS);
    }
}

void DisplayTask_Process(void)
{
    uint8_t confirm_pressed = Menu_KeyPressed(&key_confirm);
    uint8_t back_pressed = Menu_KeyPressed(&key_back);

    Menu_ForwardGrayKey();
    (void)Menu_KeyPressed(&key_up);
    (void)Menu_KeyPressed(&key_down);

    if (menu_state == MENU_STATE_RUNNING) {
        if (back_pressed != 0U) {
            Tracking_Stop();
            menu_state = MENU_STATE_SELECT;
            Menu_RenderSelect();
        } else {
            Menu_UpdateRunning();
        }
        return;
    }

    if (menu_state == MENU_STATE_SELECT) {
        if (confirm_pressed != 0U) {
            Menu_StartTracking();
        }
        return;
    }

    if (confirm_pressed != 0U) {
        Menu_StartTracking();
    } else if (back_pressed != 0U) {
        menu_state = MENU_STATE_SELECT;
        Menu_RenderSelect();
    }
}

void DisplayTask_ReportTrackingFinished(void)
{
    tracking_finished = 1U;
}
