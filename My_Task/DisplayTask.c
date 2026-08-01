#include "DisplayTask.h"

#include "OLED.h"
#include "StepMotor_Ctrl.h"
#include "cmsis_os2.h"
#include "linetrack.h"
#include "main.h"

#define MENU_SCAN_PERIOD_MS              10U
#define MENU_REFRESH_PERIOD_MS           100U
#define MENU_DEBOUNCE_SAMPLES            3U
#define MOTOR_ZERO_ADJUST_STEP_DEG        0.2f

#define QUESTION3_POSITIVE_X             400U
#define QUESTION3_NEGATIVE_X             189
#define QUESTION3_CENTER_TOLERANCE_PX    22U
#define QUESTION3_REACH_TOLERANCE_PX     11U
#define QUESTION3_STABLE_TOLERANCE_PX    22U
#define QUESTION3_STABLE_HOLD_MS         500U
#define QUESTION3_TIMEOUT_MS             0U

#define QUESTION4_CENTER_TOLERANCE_PX    22U
#define QUESTION4_TIMEOUT_MS             0U

typedef enum {
    MENU_STATE_SELECT = 0,
    MENU_STATE_RUNNING,
    MENU_STATE_COMPLETE,
    MENU_STATE_MOTOR_ZERO
} MenuState_t;

typedef enum {
    MOTOR_ZERO_PHASE_ADJUSTING = 0,
    MOTOR_ZERO_PHASE_WAIT_SAVE_START,
    MOTOR_ZERO_PHASE_WAIT_SAVE_COMPLETE,
    MOTOR_ZERO_PHASE_WAIT_READY
} MotorZeroPhase_t;

typedef enum {
    MENU_ITEM_R2_LINE_TRACK = 0,
    MENU_ITEM_R3_BALL_SWEEP,
    MENU_ITEM_R4_A_TO_B,
    MENU_ITEM_R5_A_TO_B,
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
static MotorZeroPhase_t motor_zero_phase;
static float motor_zero_adjust_angle_deg;
static Question3Stage_t question3_stage;
static uint8_t question3_stable_active;
static uint32_t task_start_tick;
static uint32_t task_finish_tick;
static uint32_t last_refresh_tick;
static uint32_t question3_stable_start_tick;
static volatile uint16_t question3_positive_x = QUESTION3_POSITIVE_X;
static volatile uint16_t question3_negative_x = QUESTION3_NEGATIVE_X;
static volatile uint16_t question3_brake_trigger_x =
    QUESTION3_BRAKE_TRIGGER_X;
static volatile float question3_brake_angle_deg =
    QUESTION3_BRAKE_ANGLE_DEG;
static volatile uint32_t question3_brake_duration_ms =
    QUESTION3_BRAKE_DURATION_MS;
static volatile uint16_t question3_brake_speed_rpm =
    QUESTION3_BRAKE_RPM;
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_BRAKE_PULSE
static uint32_t question3_brake_start_tick;
static uint8_t question3_brake_used;
#endif
static volatile uint64_t question2_odometer_target =
    QUESTION2_ODOMETER_TARGET_DEFAULT;
static volatile uint64_t question4_odometer_target =
    QUESTION4_ODOMETER_TARGET_DEFAULT;
static volatile float question4_drive_acceleration =
    QUESTION4_DRIVE_ACCEL_DEFAULT;
static uint16_t question4_max_ball_error_px;

static uint8_t Menu_KeyRead(const MenuKey_t *key)
{
    return (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint16_t Menu_AbsDiffU16(uint16_t left, uint16_t right)
{
    return (left >= right) ? (left - right) : (right - left);
}

static uint8_t Menu_IsQuestionABItem(MenuItem_t item)
{
    return ((item == MENU_ITEM_R4_A_TO_B) ||
            (item == MENU_ITEM_R5_A_TO_B)) ? 1U : 0U;
}

static uint64_t Menu_GetQuestionABOdometerTarget(MenuItem_t item)
{
    if (item == MENU_ITEM_R5_A_TO_B) {
        return DisplayTask_GetQuestion2OdometerTarget();
    }
    return DisplayTask_GetQuestion4OdometerTarget();
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
    OLED_ShowString(0U, 4U,
                    (uint8_t *)((selected_item == MENU_ITEM_R5_A_TO_B) ?
                                "> R5 A TO B" : "  R5 A TO B"),
                    8U);
    OLED_ShowString(
        0U, 5U,
        (uint8_t *)((selected_item == MENU_ITEM_R2_LINE_TRACK) ?
                    "K1:GRAY K2:NEXT" : "K1:ZERO K2:NEXT"),
        8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY3: START", 8U);
    OLED_ShowString(0U, 7U, (uint8_t *)"KEY4: STOP", 8U);
}

static void Menu_RenderMotorZeroAdjust(void)
{
    int32_t angle_tenths;
    uint32_t angle_magnitude;

    angle_tenths = (motor_zero_adjust_angle_deg >= 0.0f) ?
        (int32_t)(motor_zero_adjust_angle_deg * 10.0f + 0.5f) :
        (int32_t)(motor_zero_adjust_angle_deg * 10.0f - 0.5f);
    angle_magnitude = (angle_tenths >= 0) ?
        (uint32_t)angle_tenths : (uint32_t)(-angle_tenths);

    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"MOTOR ZERO ADJ", 8U);
    OLED_Printf(0U, 2U, 8U, "ANGLE:%c%2lu.%1lu",
                (angle_tenths < 0) ? '-' : '+',
                (unsigned long)(angle_magnitude / 10U),
                (unsigned long)(angle_magnitude % 10U));
    OLED_ShowString(0U, 4U, (uint8_t *)"K1:-0.2 K2:+0.2", 8U);
    OLED_ShowString(0U, 6U, (uint8_t *)"KEY3: SAVE", 8U);
    OLED_ShowString(0U, 7U, (uint8_t *)"KEY4: CANCEL", 8U);
}

static void Menu_RenderMotorZeroProgress(const char *status)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)"MOTOR ZERO ADJ", 8U);
    OLED_ShowString(0U, 2U, (uint8_t *)status, 8U);
}

static void Menu_StartMotorZeroAdjust(void)
{
    if (StepMotor_GetState() != STEPMOTOR_STATE_READY) {
        OLED_ShowString(0U, 4U, (uint8_t *)"MOTOR NOT READY ", 8U);
        return;
    }

    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
    motor_zero_adjust_angle_deg = StepMotor_GetTargetAngle();
    if (motor_zero_adjust_angle_deg < STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG) {
        motor_zero_adjust_angle_deg = STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG;
    } else if (motor_zero_adjust_angle_deg >
               STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) {
        motor_zero_adjust_angle_deg = STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG;
    }
    StepMotor_SetAngleOverride(true, motor_zero_adjust_angle_deg);
    motor_zero_phase = MOTOR_ZERO_PHASE_ADJUSTING;
    menu_state = MENU_STATE_MOTOR_ZERO;
    Menu_RenderMotorZeroAdjust();
}

static void Menu_AdjustMotorZero(float delta_deg)
{
    motor_zero_adjust_angle_deg += delta_deg;
    if (motor_zero_adjust_angle_deg < STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG) {
        motor_zero_adjust_angle_deg = STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG;
    } else if (motor_zero_adjust_angle_deg >
               STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) {
        motor_zero_adjust_angle_deg = STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG;
    }
    StepMotor_SetAngleOverride(true, motor_zero_adjust_angle_deg);
    Menu_RenderMotorZeroAdjust();
}

static void Menu_UpdateMotorZero(uint8_t decrease_pressed,
                                 uint8_t increase_pressed,
                                 uint8_t save_pressed,
                                 uint8_t cancel_pressed)
{
    StepMotorControlState_t motor_state = StepMotor_GetState();

    if (motor_zero_phase == MOTOR_ZERO_PHASE_ADJUSTING) {
        if (decrease_pressed != 0U) {
            Menu_AdjustMotorZero(-MOTOR_ZERO_ADJUST_STEP_DEG);
        } else if (increase_pressed != 0U) {
            Menu_AdjustMotorZero(MOTOR_ZERO_ADJUST_STEP_DEG);
        } else if (save_pressed != 0U) {
            if (StepMotor_SetEnabled(false)) {
                motor_zero_phase = MOTOR_ZERO_PHASE_WAIT_SAVE_START;
                Menu_RenderMotorZeroProgress("SAVING ZERO");
            } else {
                OLED_ShowString(0U, 3U, (uint8_t *)"MOTOR BUSY", 8U);
            }
        } else if (cancel_pressed != 0U) {
            StepMotor_SetAngleOverride(false, 0.0f);
            menu_state = MENU_STATE_SELECT;
            Menu_RenderSelect();
        }
        return;
    }

    if (motor_zero_phase == MOTOR_ZERO_PHASE_WAIT_SAVE_START) {
        if ((motor_state == STEPMOTOR_STATE_DISABLED) &&
            StepMotor_SaveCurrentAsHome()) {
            motor_zero_phase = MOTOR_ZERO_PHASE_WAIT_SAVE_COMPLETE;
        }
        return;
    }

    if (motor_zero_phase == MOTOR_ZERO_PHASE_WAIT_SAVE_COMPLETE) {
        if ((motor_state == STEPMOTOR_STATE_DISABLED) &&
            StepMotor_SetEnabled(true)) {
            motor_zero_phase = MOTOR_ZERO_PHASE_WAIT_READY;
            Menu_RenderMotorZeroProgress("HOMING");
        }
        return;
    }

    if ((motor_zero_phase == MOTOR_ZERO_PHASE_WAIT_READY) &&
        (motor_state == STEPMOTOR_STATE_READY)) {
        menu_state = MENU_STATE_SELECT;
        Menu_RenderSelect();
    }
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
    uint16_t center_x = StepMotor_GetCameraCenterX();
    uint64_t odometer = Tracking_GetOdometerCounts();
    uint64_t target = Menu_GetQuestionABOdometerTarget(running_item);

    OLED_Printf(0U, 2U, 8U, "X:%3u T:%3u   ",
                (unsigned int)ball_x,
                (unsigned int)center_x);
    OLED_Printf(0U, 3U, 8U, "O:%lu/%lu   ",
                (unsigned long)odometer, (unsigned long)target);
    OLED_Printf(0U, 4U, 8U, "TIME:%2lu.%1lu S ",
                (unsigned long)(elapsed_ms / 1000U),
                (unsigned long)((elapsed_ms % 1000U) / 100U));
    OLED_Printf(0U, 5U, 8U, "PWM:%3lu A:%4lu",
                (unsigned long)TrackingQuestion4Tune.base_pwm,
                (unsigned long)question4_drive_acceleration);
}

static void Menu_RenderQuestion4Running(void)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U,
                    (uint8_t *)((running_item == MENU_ITEM_R5_A_TO_B) ?
                                "R5 A TO B" : "R4 A TO B"),
                    8U);
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
    if (running_item == MENU_ITEM_R4_A_TO_B) {
        return "R4 A TO B";
    }
    return "R5 A TO B";
}

static void Menu_RenderResult(const char *status, uint32_t elapsed_ms)
{
    OLED_Clear();
    OLED_ShowString(0U, 0U, (uint8_t *)Menu_GetRunningTitle(), 8U);
    OLED_Printf(0U, 1U, 8U, "STATUS: %s", status);
    if (Menu_IsQuestionABItem(running_item) != 0U) {
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
    Tracking_SetTune(&TrackingQuestion2Tune);
    Tracking_SetDriveAcceleration(0.0f);
    Tracking_SetOdometerTarget(DisplayTask_GetQuestion2OdometerTarget());
    task_start_tick = HAL_GetTick();
    last_refresh_tick = task_start_tick;
    menu_state = MENU_STATE_RUNNING;
    Tracking_Start();
    Menu_RenderTrackingRunning();
}

static void Menu_StartQuestion3(void)
{
    uint16_t ball_x;
    uint16_t center_x;
    uint16_t positive_x;

    if (StepMotor_GetState() != STEPMOTOR_STATE_READY) {
        OLED_ShowString(0U, 4U, (uint8_t *)"MOTOR NOT READY ", 8U);
        return;
    }

    ball_x = StepMotor_GetBallX();
    center_x = StepMotor_GetCameraCenterX();
    if (Menu_AbsDiffU16(ball_x, center_x) >
        QUESTION3_CENTER_TOLERANCE_PX) {
        (void)StepMotor_SetBallTargetX(center_x);
        OLED_ShowString(0U, 4U, (uint8_t *)"MOVE BALL TO O  ", 8U);
        OLED_Printf(0U, 3U, 8U, "X:%3u O:%3u   ",
                    (unsigned int)ball_x,
                    (unsigned int)center_x);
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
    positive_x = DisplayTask_GetQuestion3PositiveX();
    (void)StepMotor_SetBallTargetX(positive_x);
    Menu_RenderQuestion3Running();
}

static void Menu_StartQuestion4(MenuItem_t item)
{
    uint16_t ball_x;
    uint16_t center_x;
    uint64_t odometer_target;

    if ((Tracking_IsReady() == 0U) ||
        (StepMotor_GetState() != STEPMOTOR_STATE_READY)) {
        OLED_ShowString(0U, 4U, (uint8_t *)"SYSTEM NOT READY", 8U);
        return;
    }

    odometer_target = Menu_GetQuestionABOdometerTarget(item);
    if (odometer_target == 0U) {
        OLED_ShowString(0U, 4U,
                        (uint8_t *)((item == MENU_ITEM_R5_A_TO_B) ?
                                    "SET Q2 ODO FIRST" :
                                    "SET Q4 ODO FIRST"),
                        8U);
        return;
    }

    ball_x = StepMotor_GetBallX();
    center_x = StepMotor_GetCameraCenterX();
    if (Menu_AbsDiffU16(ball_x, center_x) >
        QUESTION4_CENTER_TOLERANCE_PX) {
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(center_x);
        OLED_ShowString(0U, 4U, (uint8_t *)"MOVE BALL TO O  ", 8U);
        OLED_Printf(0U, 3U, 8U, "X:%3u O:%3u   ",
                    (unsigned int)ball_x,
                    (unsigned int)center_x);
        return;
    }

    running_item = item;
    tracking_finished = 0U;
    question4_max_ball_error_px =
        Menu_AbsDiffU16(ball_x, center_x);
    Tracking_SetTune(&TrackingQuestion4Tune);
    Tracking_SetOdometerTarget(odometer_target);
    Tracking_SetDriveAcceleration(question4_drive_acceleration);
    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_QUESTION4);
    (void)StepMotor_SetBallTargetX(center_x);
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
    } else if (Menu_IsQuestionABItem(selected_item) != 0U) {
        Menu_StartQuestion4(selected_item);
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
    uint16_t positive_x = DisplayTask_GetQuestion3PositiveX();
    uint16_t negative_x = DisplayTask_GetQuestion3NegativeX();

    if ((QUESTION3_TIMEOUT_MS != 0U) &&
        (elapsed >= QUESTION3_TIMEOUT_MS)) {
        (void)StepMotor_SetBallTargetX(negative_x);
        Menu_CompleteQuestion3("TIMEOUT", now);
        return;
    }

    if (question3_stage == QUESTION3_STAGE_TO_POSITIVE) {
        if (Menu_AbsDiffU16(ball_x, positive_x) <=
            QUESTION3_REACH_TOLERANCE_PX) {
#if QUESTION3_CONTROL_STRATEGY == QUESTION3_STRATEGY_DUAL_PID
            (void)StepMotor_SetControlProfile(
                STEPMOTOR_PROFILE_QUESTION3_NEGATIVE);
#endif
            (void)StepMotor_SetBallTargetX(negative_x);
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
        StepMotor_SetAngleOverrideWithSpeed(true,
                                            question3_brake_angle_deg,
                                            question3_brake_speed_rpm);
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
        if (Menu_AbsDiffU16(ball_x, negative_x) <=
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
    Tracking_SetTune(&TrackingQuestion2Tune);
    Tracking_SetOdometerTarget(DisplayTask_GetQuestion2OdometerTarget());
    Tracking_SetDriveAcceleration(0.0f);
    StepMotor_SetAngleOverride(false, 0.0f);
    (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
    (void)StepMotor_SetBallTargetX(StepMotor_GetCameraCenterX());
    task_finish_tick = now;
    menu_state = MENU_STATE_COMPLETE;
    Menu_RenderResult(status, task_finish_tick - task_start_tick);
}

static void Menu_UpdateQuestion4(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - task_start_tick;
    uint16_t ball_error = Menu_AbsDiffU16(
        StepMotor_GetBallX(), StepMotor_GetCameraCenterX());

    if (ball_error > question4_max_ball_error_px) {
        question4_max_ball_error_px = ball_error;
    }

    if (tracking_finished != 0U) {
        const char *status;

        tracking_finished = 0U;
        if ((QUESTION4_TIMEOUT_MS != 0U) &&
            (elapsed > QUESTION4_TIMEOUT_MS)) {
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

    if ((QUESTION4_TIMEOUT_MS != 0U) &&
        (elapsed >= QUESTION4_TIMEOUT_MS)) {
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
        (void)StepMotor_SetBallTargetX(StepMotor_GetCameraCenterX());
    } else {
        Tracking_Stop();
        Tracking_SetTune(&TrackingQuestion2Tune);
        Tracking_SetOdometerTarget(DisplayTask_GetQuestion2OdometerTarget());
        Tracking_SetDriveAcceleration(0.0f);
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(StepMotor_GetCameraCenterX());
    }
    menu_state = MENU_STATE_SELECT;
    Menu_RenderSelect();
}

static void Menu_ReturnFromResult(void)
{
    if ((running_item == MENU_ITEM_R3_BALL_SWEEP) ||
        (Menu_IsQuestionABItem(running_item) != 0U)) {
        StepMotor_SetAngleOverride(false, 0.0f);
        (void)StepMotor_SetControlProfile(STEPMOTOR_PROFILE_DEFAULT);
        (void)StepMotor_SetBallTargetX(StepMotor_GetCameraCenterX());
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
    uint8_t up_pressed;

    Menu_ForwardGrayKey();
    up_pressed = Menu_KeyPressed(&key_up);

    if (menu_state == MENU_STATE_MOTOR_ZERO) {
        Menu_UpdateMotorZero(up_pressed,
                             down_pressed,
                             confirm_pressed,
                             back_pressed);
        return;
    }

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
        } else if ((up_pressed != 0U) &&
                   (selected_item != MENU_ITEM_R2_LINE_TRACK)) {
            Menu_StartMotorZeroAdjust();
        }
        return;
    }

    if (confirm_pressed != 0U) {
        if (running_item == MENU_ITEM_R2_LINE_TRACK) {
            Menu_StartTracking();
        } else if (Menu_IsQuestionABItem(running_item) != 0U) {
            Menu_StartQuestion4(running_item);
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

void DisplayTask_SetQuestion3PositiveX(uint16_t target_x)
{
    if (target_x <= STEPMOTOR_CAMERA_X_MAX) {
        question3_positive_x = target_x;
    }
}

void DisplayTask_SetQuestion3NegativeX(uint16_t target_x)
{
    if (target_x <= STEPMOTOR_CAMERA_X_MAX) {
        question3_negative_x = target_x;
    }
}

uint16_t DisplayTask_GetQuestion3PositiveX(void)
{
    return question3_positive_x;
}

uint16_t DisplayTask_GetQuestion3NegativeX(void)
{
    return question3_negative_x;
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

void DisplayTask_SetQuestion3BrakeSpeed(uint16_t speed_rpm)
{
    question3_brake_speed_rpm = speed_rpm;
}

uint16_t DisplayTask_GetQuestion3BrakeTriggerX(void)
{
    return question3_brake_trigger_x;
}

float DisplayTask_GetQuestion3BrakeAngle(void)
{
    return question3_brake_angle_deg;
}

uint32_t DisplayTask_GetQuestion3BrakeDuration(void)
{
    return question3_brake_duration_ms;
}

uint16_t DisplayTask_GetQuestion3BrakeSpeed(void)
{
    return question3_brake_speed_rpm;
}

void DisplayTask_SetQuestion2OdometerTarget(uint64_t target_counts)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    question2_odometer_target = target_counts;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint64_t DisplayTask_GetQuestion2OdometerTarget(void)
{
    uint64_t target;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    target = question2_odometer_target;
    if (primask == 0U) {
        __enable_irq();
    }
    return target;
}

void DisplayTask_SetQuestion4DrivePWM(float drive_pwm)
{
    if (drive_pwm < 0.0f) {
        drive_pwm = 0.0f;
    } else if (drive_pwm > TRACKING_PWM_MAX) {
        drive_pwm = TRACKING_PWM_MAX;
    }
    TrackingQuestion4Tune.base_pwm = drive_pwm;
}

void DisplayTask_SetQuestion4DriveAcceleration(float pwm_per_second)
{
    if (pwm_per_second < 0.0f) {
        pwm_per_second = 0.0f;
    } else if (pwm_per_second > QUESTION4_DRIVE_ACCEL_MAX) {
        pwm_per_second = QUESTION4_DRIVE_ACCEL_MAX;
    }
    question4_drive_acceleration = pwm_per_second;
}

float DisplayTask_GetQuestion4DrivePWM(void)
{
    return TrackingQuestion4Tune.base_pwm;
}

float DisplayTask_GetQuestion4DriveAcceleration(void)
{
    return question4_drive_acceleration;
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
