#include "DisplayTask.h"

#include "OLED.h"
#include "StepMotor_Ctrl.h"
#include "cmsis_os2.h"
#include "linetrack.h"
#include "main.h"

#define MENU_SCAN_PERIOD_MS              10U
#define MENU_REFRESH_PERIOD_MS           100U
#define MENU_DEBOUNCE_SAMPLES            3U

#define QUESTION3_POSITIVE_X             400U
#define QUESTION3_NEGATIVE_X             189
#define QUESTION3_CENTER_TOLERANCE_PX    22U
#define QUESTION3_REACH_TOLERANCE_PX     11U
#define QUESTION3_STABLE_TOLERANCE_PX    22U
#define QUESTION3_STABLE_HOLD_MS         500U
#define QUESTION3_TIMEOUT_MS             5000U

#define QUESTION4_CENTER_TOLERANCE_PX    22U
#define QUESTION4_TIMEOUT_MS             8000U

typedef enum {
    MENU_STATE_SELECT = 0,
    MENU_STATE_RUNNING,
    MENU_STATE_COMPLETE
} MenuState_t;

typedef enum {
    MENU_ITEM_R2_LINE_TRACK = 0,
    MENU_ITEM_R3_BALL_SWEEP,
    MENU_ITEM_R4_A_TO_B,
    MENU_ITEM_COUNT
} MenuItem_t;

typedef enum {
    QUESTION3_STAGE_TO_POSITIVE = 0,
    QUESTION3_STAGE_TO_NEGATIVE,
    QUESTION3_STAGE_BRAKING,
    QUESTION3_STAGE_STABILIZING
} Question3Stage_t;

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
static MenuItem_t selected_item = MENU_ITEM_R2_LINE_TRACK;
static MenuItem_t running_item = MENU_ITEM_R2_LINE_TRACK;
static Question3Stage_t question3_stage;
static uint8_t question3_stable_active;
static uint32_t task_start_tick;
static uint32_t task_finish_tick;
static uint32_t last_refresh_tick;
static uint32_t question3_stable_start_tick;
static volatile uint16_t question3_brake_trigger_x =
    QUESTION3_BRAKE_TRIGGER_X;
static volatile float question3_brake_angle_deg =
    QUESTION3_BRAKE_ANGLE_DEG;
static volatile uint32_t question3_brake_duration_ms =
    QUESTION3_BRAKE_DURATION_MS;
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_BRAKE_PULSE
static uint32_t question3_brake_start_tick;
static uint8_t question3_brake_used;
#endif
static volatile uint64_t question4_odometer_target =
    QUESTION4_ODOMETER_TARGET_DEFAULT;
static uint64_t question4_previous_odometer_target;
static uint16_t question4_max_ball_error_px;

static uint8_t Menu_KeyRead(const MenuKey_t *key)
{
    return (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint16_t Menu_AbsDiffU16(uint16_t left, uint16_t right)
{
    return (left >= right) ? (left - right) : (right - left);
}

static void Menu_ForwardGrayKey(void)
{
    GPIO_PinState output_state = GPIO_PIN_SET;

    if ((menu_state == MENU_STATE_SELECT) &&
        (selected_item == MENU_ITEM_R2_LINE_TRACK) &&
        (Menu_KeyRead(&key_up) != 0U)) {
        output_state = GPIO_PIN_RESET;
    }
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
    OLED_ShowString(0U, 1U,
                    (uint8_t *)((selected_item == MENU_ITEM_R2_LINE_TRACK) ?
                                "> R2 LINE TRACK" : "  R2 LINE TRACK"),
                    8U);
    OLED_ShowString(0U, 2U,
                    (uint8_t *)((selected_item == MENU_ITEM_R3_BALL_SWEEP) ?
                                "> R3 BALL +/-5" : "  R3 BALL +/-5"),
                    8U);
    OLED_ShowString(0U, 3U,
                    (uint8_t *)((selected_item == MENU_ITEM_R4_A_TO_B) ?
                                "> R4 A TO B" : "  R4 A TO B"),
                    8U);
    OLED_ShowString(0U, 5U, (uint8_t *)"K1:CAL K2:NEXT", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY3: START", 8U);
    OLED_ShowString(0U, 7U, (uint8_t *)"KEY4: STOP", 8U);
}

static void Menu_RenderTrackingRunning(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"R2 LINE TRACK", 8U);
    OLED_ShowString(0U, 1U, (uint8_t *)"STATUS: RUNNING", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: ABORT", 8U);
}

static void Menu_RenderQuestion3Status(uint32_t elapsed_ms)
{
    uint16_t ball_x = StepMotor_GetBallX();
    uint16_t target_x = StepMotor_GetBallTargetX();

    if (question3_stage == QUESTION3_STAGE_TO_POSITIVE) {
        OLED_ShowString(0U, 1U, (uint8_t *)"STAGE: TO +5CM  ", 8U);
    } else if (question3_stage == QUESTION3_STAGE_TO_NEGATIVE) {
        OLED_ShowString(0U, 1U, (uint8_t *)"STAGE: TO -5CM  ", 8U);
    } else if (question3_stage == QUESTION3_STAGE_BRAKING) {
        OLED_ShowString(0U, 1U, (uint8_t *)"STAGE: BRAKING  ", 8U);
    } else {
        OLED_ShowString(0U, 1U, (uint8_t *)"STAGE: STABLE   ", 8U);
    }
    OLED_Printf(0U, 2U, 8U, "X:%3u T:%3u   ",
                (unsigned int)ball_x, (unsigned int)target_x);
    OLED_Printf(0U, 3U, 8U, "TIME:%2lu.%1lu S ",
                (unsigned long)(elapsed_ms / 1000U),
                (unsigned long)((elapsed_ms % 1000U) / 100U));
}

static void Menu_RenderQuestion3Running(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"R3 BALL +/-5CM", 8U);
    Menu_RenderQuestion3Status(0U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: ABORT", 8U);
}

static void Menu_RenderQuestion4Status(uint32_t elapsed_ms)
{
    uint16_t ball_x = StepMotor_GetBallX();
    uint64_t odometer = Tracking_GetOdometerCounts();
    uint64_t target = DisplayTask_GetQuestion4OdometerTarget();

    OLED_Printf(0U, 2U, 8U, "X:%3u T:%3u   ",
                (unsigned int)ball_x,
                (unsigned int)STEPMOTOR_CAMERA_CENTER_X);
    OLED_Printf(0U, 3U, 8U, "O:%lu/%lu   ",
                (unsigned long)odometer, (unsigned long)target);
    OLED_Printf(0U, 4U, 8U, "TIME:%2lu.%1lu S ",
                (unsigned long)(elapsed_ms / 1000U),
                (unsigned long)((elapsed_ms % 1000U) / 100U));
}

static void Menu_RenderQuestion4Running(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"R4 A TO B", 8U);
    OLED_ShowString(0U, 1U, (uint8_t *)"STATUS: RUNNING", 8U);
    Menu_RenderQuestion4Status(0U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: ABORT", 8U);
}

static const char *Menu_GetRunningTitle(void)
{
    if (running_item == MENU_ITEM_R2_LINE_TRACK) {
        return "R2 LINE TRACK";
    }
    if (running_item == MENU_ITEM_R3_BALL_SWEEP) {
        return "R3 BALL +/-5CM";
    }
    return "R4 A TO B";
}

static void Menu_RenderResult(const char *status, uint32_t elapsed_ms)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)Menu_GetRunningTitle(), 8U);
    OLED_Printf(0U, 1U, 8U, "STATUS: %s", status);
    if (running_item == MENU_ITEM_R4_A_TO_B) {
        OLED_Printf(0U, 2U, 8U, "MAX ERR:%3uPX  ",
                    (unsigned int)question4_max_ball_error_px);
    }
    OLED_Printf(0U, 3U, 8U, "TIME: %2lu.%1lu S",
                (unsigned long)(elapsed_ms / 1000U),
                (unsigned long)((elapsed_ms % 1000U) / 100U));
    if (running_item != MENU_ITEM_R3_BALL_SWEEP) {
        OLED_ShowString(0U, 5U, (uint8_t *)"KEY3: AGAIN", 8U);
    } else {
        OLED_ShowString(0U, 5U, (uint8_t *)"KEY3: CENTER", 8U);
    }
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY4: MENU", 8U);
}

static void Menu_StartTracking(void)
{
    if (Tracking_IsReady() == 0U) {
        OLED_ShowString(0U, 4U, (uint8_t *)"SENSOR NOT READY", 8U);
        return;
    }

    tracking_finished = 0U;
    running_item = MENU_ITEM_R2_LINE_TRACK;
    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
    task_start_tick = HAL_GetTick();
    last_refresh_tick = task_start_tick;
    menu_state = MENU_STATE_RUNNING;
    Tracking_Start();
    Menu_RenderTrackingRunning();
}

static void Menu_StartQuestion3(void)
{
    uint16_t ball_x;

    if (StepMotor_GetState() != STEPMOTOR_STATE_READY) {
        OLED_ShowString(0U, 4U, (uint8_t *)"MOTOR NOT READY ", 8U);
        return;
    }

    ball_x = StepMotor_GetBallX();
    if (Menu_AbsDiffU16(ball_x, STEPMOTOR_CAMERA_CENTER_X) >
        QUESTION3_CENTER_TOLERANCE_PX) {
        (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
        OLED_ShowString(0U, 4U, (uint8_t *)"MOVE BALL TO O  ", 8U);
        OLED_Printf(0U, 3U, 8U, "X:%3u O:%3u   ",
                    (unsigned int)ball_x,
                    (unsigned int)STEPMOTOR_CAMERA_CENTER_X);
        return;
    }

    if (Tracking_IsReady() != 0U) {
        Tracking_Stop();
    }
    running_item = MENU_ITEM_R3_BALL_SWEEP;
    StepMotor_SetAngleOverride(false, 0.0f);
    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_QUESTION3);
    question3_stage = QUESTION3_STAGE_TO_POSITIVE;
    question3_stable_active = 0U;
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_BRAKE_PULSE
    question3_brake_used = 0U;
#endif
    task_start_tick = HAL_GetTick();
    last_refresh_tick = task_start_tick;
    menu_state = MENU_STATE_RUNNING;
    (void)StepMotor_SetBallTargetX(QUESTION3_POSITIVE_X);
    Menu_RenderQuestion3Running();
}

static void Menu_StartQuestion4(void)
{
    uint16_t ball_x;
    uint64_t odometer_target;

    if ((Tracking_IsReady() == 0U) ||
        (StepMotor_GetState() != STEPMOTOR_STATE_READY)) {
        OLED_ShowString(0U, 4U, (uint8_t *)"SYSTEM NOT READY", 8U);
        return;
    }

    odometer_target = DisplayTask_GetQuestion4OdometerTarget();
    if (odometer_target == 0U) {
        OLED_ShowString(0U, 4U, (uint8_t *)"SET Q4 ODO FIRST", 8U);
        return;
    }

    ball_x = StepMotor_GetBallX();
    if (Menu_AbsDiffU16(ball_x, STEPMOTOR_CAMERA_CENTER_X) >
        QUESTION4_CENTER_TOLERANCE_PX) {
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
        OLED_ShowString(0U, 4U, (uint8_t *)"MOVE BALL TO O  ", 8U);
        OLED_Printf(0U, 3U, 8U, "X:%3u O:%3u   ",
                    (unsigned int)ball_x,
                    (unsigned int)STEPMOTOR_CAMERA_CENTER_X);
        return;
    }

    running_item = MENU_ITEM_R4_A_TO_B;
    tracking_finished = 0U;
    question4_max_ball_error_px =
        Menu_AbsDiffU16(ball_x, STEPMOTOR_CAMERA_CENTER_X);
    question4_previous_odometer_target = Tracking_GetOdometerTarget();
    Tracking_SetOdometerTarget(odometer_target);
    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_QUESTION4);
    (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
    task_start_tick = HAL_GetTick();
    last_refresh_tick = task_start_tick;
    menu_state = MENU_STATE_RUNNING;
    Tracking_Start();
    Menu_RenderQuestion4Running();
}

static void Menu_StartSelected(void)
{
    if (selected_item == MENU_ITEM_R2_LINE_TRACK) {
        Menu_StartTracking();
    } else if (selected_item == MENU_ITEM_R3_BALL_SWEEP) {
        Menu_StartQuestion3();
    } else {
        Menu_StartQuestion4();
    }
}

static void Menu_UpdateTracking(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - task_start_tick;

    if (tracking_finished != 0U) {
        tracking_finished = 0U;
        task_finish_tick = now;
        menu_state = MENU_STATE_COMPLETE;
        Menu_RenderResult("COMPLETE", task_finish_tick - task_start_tick);
        return;
    }
    if ((now - last_refresh_tick) >= MENU_REFRESH_PERIOD_MS) {
        last_refresh_tick = now;
        OLED_Printf(0U, 3U, 8U, "TIME: %2lu.%1lu S",
                    (unsigned long)(elapsed / 1000U),
                    (unsigned long)((elapsed % 1000U) / 100U));
    }
}

static void Menu_CompleteQuestion3(const char *status, uint32_t now)
{
    StepMotor_SetAngleOverride(false, 0.0f);
    task_finish_tick = now;
    menu_state = MENU_STATE_COMPLETE;
    Menu_RenderResult(status, task_finish_tick - task_start_tick);
}

static void Menu_UpdateQuestion3(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - task_start_tick;
    uint16_t ball_x = StepMotor_GetBallX();

    if (elapsed >= QUESTION3_TIMEOUT_MS) {
        (void)StepMotor_SetBallTargetX(QUESTION3_NEGATIVE_X);
        Menu_CompleteQuestion3("TIMEOUT", now);
        return;
    }

    if (question3_stage == QUESTION3_STAGE_TO_POSITIVE) {
        if (Menu_AbsDiffU16(ball_x, QUESTION3_POSITIVE_X) <=
            QUESTION3_REACH_TOLERANCE_PX) {
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_DUAL_PID
            (void)StepMotor_SetControlProfile(
                STEPMOTOR_PROFILE_QUESTION3_NEGATIVE);
#endif
            (void)StepMotor_SetBallTargetX(QUESTION3_NEGATIVE_X);
            question3_stage = QUESTION3_STAGE_TO_NEGATIVE;
            question3_stable_active = 0U;
        }
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_BRAKE_PULSE
    } else if ((question3_stage == QUESTION3_STAGE_TO_NEGATIVE) &&
               (question3_brake_used == 0U) &&
               (ball_x <= question3_brake_trigger_x)) {
        question3_brake_used = 1U;
        question3_brake_start_tick = now;
        question3_stage = QUESTION3_STAGE_BRAKING;
        question3_stable_active = 0U;
        StepMotor_SetAngleOverride(true, question3_brake_angle_deg);
    } else if (question3_stage == QUESTION3_STAGE_BRAKING) {
        if ((now - question3_brake_start_tick) >=
            question3_brake_duration_ms) {
            StepMotor_SetAngleOverride(false, 0.0f);
            (void)StepMotor_SetControlProfile(
                STEPMOTOR_PROFILE_QUESTION3_NEGATIVE);
            question3_stage = QUESTION3_STAGE_TO_NEGATIVE;
        }
#endif
    } else {
        if (Menu_AbsDiffU16(ball_x, QUESTION3_NEGATIVE_X) <=
            QUESTION3_STABLE_TOLERANCE_PX) {
            if (question3_stable_active == 0U) {
                question3_stable_active = 1U;
                question3_stable_start_tick = now;
                question3_stage = QUESTION3_STAGE_STABILIZING;
            } else if ((now - question3_stable_start_tick) >=
                       QUESTION3_STABLE_HOLD_MS) {
                Menu_CompleteQuestion3("COMPLETE", now);
                return;
            }
        } else {
            question3_stable_active = 0U;
            question3_stage = QUESTION3_STAGE_TO_NEGATIVE;
        }
    }

    if ((now - last_refresh_tick) >= MENU_REFRESH_PERIOD_MS) {
        last_refresh_tick = now;
        Menu_RenderQuestion3Status(elapsed);
    }
}

static void Menu_CompleteQuestion4(const char *status, uint32_t now)
{
    Tracking_SetOdometerTarget(question4_previous_odometer_target);
    task_finish_tick = now;
    menu_state = MENU_STATE_COMPLETE;
    Menu_RenderResult(status, task_finish_tick - task_start_tick);
}

static void Menu_UpdateQuestion4(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - task_start_tick;
    uint16_t ball_error = Menu_AbsDiffU16(
        StepMotor_GetBallX(), STEPMOTOR_CAMERA_CENTER_X);

    if (ball_error > question4_max_ball_error_px) {
        question4_max_ball_error_px = ball_error;
    }

    if (tracking_finished != 0U) {
        const char *status;

        tracking_finished = 0U;
        if (elapsed > QUESTION4_TIMEOUT_MS) {
            status = "TIMEOUT";
        } else if (question4_max_ball_error_px >
                   QUESTION4_CENTER_TOLERANCE_PX) {
            status = "BALL ERR";
        } else {
            status = "COMPLETE";
        }
        Menu_CompleteQuestion4(status, now);
        return;
    }

    if (elapsed >= QUESTION4_TIMEOUT_MS) {
        Tracking_Stop();
        Menu_CompleteQuestion4("TIMEOUT", now);
        return;
    }

    if ((now - last_refresh_tick) >= MENU_REFRESH_PERIOD_MS) {
        last_refresh_tick = now;
        Menu_RenderQuestion4Status(elapsed);
    }
}

static void Menu_AbortRunning(void)
{
    if (running_item == MENU_ITEM_R2_LINE_TRACK) {
        Tracking_Stop();
    } else if (running_item == MENU_ITEM_R3_BALL_SWEEP) {
        StepMotor_SetAngleOverride(false, 0.0f);
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
    } else {
        Tracking_Stop();
        Tracking_SetOdometerTarget(question4_previous_odometer_target);
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
    }
    menu_state = MENU_STATE_SELECT;
    Menu_RenderSelect();
}

static void Menu_ReturnFromResult(void)
{
    if ((running_item == MENU_ITEM_R3_BALL_SWEEP) ||
        (running_item == MENU_ITEM_R4_A_TO_B)) {
        StepMotor_SetAngleOverride(false, 0.0f);
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(STEPMOTOR_CAMERA_CENTER_X);
    }
    menu_state = MENU_STATE_SELECT;
    Menu_RenderSelect();
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
    uint8_t down_pressed = Menu_KeyPressed(&key_down);
    uint8_t confirm_pressed = Menu_KeyPressed(&key_confirm);
    uint8_t back_pressed = Menu_KeyPressed(&key_back);

    Menu_ForwardGrayKey();
    (void)Menu_KeyPressed(&key_up);

    if (menu_state == MENU_STATE_RUNNING) {
        if (back_pressed != 0U) {
            Menu_AbortRunning();
        } else if (running_item == MENU_ITEM_R2_LINE_TRACK) {
            Menu_UpdateTracking();
        } else if (running_item == MENU_ITEM_R3_BALL_SWEEP) {
            Menu_UpdateQuestion3();
        } else {
            Menu_UpdateQuestion4();
        }
        return;
    }

    if (menu_state == MENU_STATE_SELECT) {
        if (down_pressed != 0U) {
            selected_item = (MenuItem_t)(((uint8_t)selected_item + 1U) %
                                         (uint8_t)MENU_ITEM_COUNT);
            Menu_RenderSelect();
        } else if (confirm_pressed != 0U) {
            Menu_StartSelected();
        }
        return;
    }

    if (confirm_pressed != 0U) {
        if (running_item == MENU_ITEM_R2_LINE_TRACK) {
            Menu_StartTracking();
        } else if (running_item == MENU_ITEM_R4_A_TO_B) {
            Menu_StartQuestion4();
        } else {
            Menu_ReturnFromResult();
        }
    } else if (back_pressed != 0U) {
        Menu_ReturnFromResult();
    }
}

void DisplayTask_ReportTrackingFinished(void)
{
    tracking_finished = 1U;
}

void DisplayTask_SetQuestion3BrakeTriggerX(uint16_t trigger_x)
{
    question3_brake_trigger_x = trigger_x;
}

void DisplayTask_SetQuestion3BrakeAngle(float angle_deg)
{
    question3_brake_angle_deg = angle_deg;
}

void DisplayTask_SetQuestion3BrakeDuration(uint32_t duration_ms)
{
    question3_brake_duration_ms = duration_ms;
}

void DisplayTask_SetQuestion4OdometerTarget(uint64_t target_counts)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    question4_odometer_target = target_counts;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint64_t DisplayTask_GetQuestion4OdometerTarget(void)
{
    uint64_t target;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    target = question4_odometer_target;
    if (primask == 0U) {
        __enable_irq();
    }
    return target;
}
