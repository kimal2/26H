#include "StepMotor_Ctrl.h"

#include "pid.h"
#include "usart.h"

#include <math.h>

#define STEPMOTOR_ABSOLUTE_POSITION_MODE      1U

StepMotor_t WaterTubeMotor;
StepMotorTune_t StepMotorTune = {
    .ball_kp = STEPMOTOR_PID_KP,
    .ball_ki = STEPMOTOR_PID_KI,
    .ball_kd = STEPMOTOR_PID_KD,
    .ball_kff = STEPMOTOR_FEEDFORWARD_KFF,
    .feedforward_lpf_alpha = STEPMOTOR_FEEDFORWARD_LPF_ALPHA,
    .ball_lpf_alpha = STEPMOTOR_BALL_X_LPF_ALPHA,
    .angle_bias_deg = 0.0f,
    .angle_min_deg = STEPMOTOR_TUBE_ANGLE_MIN_DEG,
    .angle_max_deg = STEPMOTOR_TUBE_ANGLE_MAX_DEG,
    .run_speed_rpm = STEPMOTOR_RUN_SPEED_RPM,
};
StepMotorTune_t StepMotorQuestion3Tune = {
    .ball_kp = STEPMOTOR_Q3_PID_KP,
    .ball_ki = STEPMOTOR_Q3_PID_KI,
    .ball_kd = STEPMOTOR_Q3_PID_KD,
    .ball_kff = STEPMOTOR_Q3_FEEDFORWARD_KFF,
    .feedforward_lpf_alpha = STEPMOTOR_Q3_FEEDFORWARD_LPF_ALPHA,
    .ball_lpf_alpha = STEPMOTOR_Q3_BALL_X_LPF_ALPHA,
    .angle_bias_deg = 0.0f,
    .angle_min_deg = STEPMOTOR_Q3_ANGLE_MIN_DEG,
    .angle_max_deg = STEPMOTOR_Q3_ANGLE_MAX_DEG,
    .run_speed_rpm = STEPMOTOR_Q3_RUN_SPEED_RPM,
};
StepMotorTune_t StepMotorQuestion3NegativeTune = {
    .ball_kp = STEPMOTOR_Q3_NEG_PID_KP,
    .ball_ki = STEPMOTOR_Q3_NEG_PID_KI,
    .ball_kd = STEPMOTOR_Q3_NEG_PID_KD,
    .ball_kff = STEPMOTOR_Q3_NEG_FEEDFORWARD_KFF,
    .feedforward_lpf_alpha = STEPMOTOR_Q3_NEG_FEEDFORWARD_LPF_ALPHA,
    .ball_lpf_alpha = STEPMOTOR_Q3_NEG_BALL_X_LPF_ALPHA,
    .angle_bias_deg = STEPMOTOR_Q3_NEG_ANGLE_BIAS_DEG,
    .angle_min_deg = STEPMOTOR_Q3_NEG_ANGLE_MIN_DEG,
    .angle_max_deg = STEPMOTOR_Q3_NEG_ANGLE_MAX_DEG,
    .run_speed_rpm = STEPMOTOR_Q3_NEG_RUN_SPEED_RPM,
};
StepMotorTune_t StepMotorQuestion4Tune = {
    .ball_kp = STEPMOTOR_Q4_PID_KP,
    .ball_ki = STEPMOTOR_Q4_PID_KI,
    .ball_kd = STEPMOTOR_Q4_PID_KD,
    .ball_kff = STEPMOTOR_Q4_FEEDFORWARD_KFF,
    .feedforward_lpf_alpha = STEPMOTOR_Q4_FEEDFORWARD_LPF_ALPHA,
    .ball_lpf_alpha = STEPMOTOR_Q4_BALL_X_LPF_ALPHA,
    .angle_bias_deg = 0.0f,
    .angle_min_deg = STEPMOTOR_Q4_ANGLE_MIN_DEG,
    .angle_max_deg = STEPMOTOR_Q4_ANGLE_MAX_DEG,
    .run_speed_rpm = STEPMOTOR_Q4_RUN_SPEED_RPM,
};
StepMotorReturnTune_t StepMotorQuestion4ReturnTune = {
    .enabled = STEPMOTOR_Q4_RETURN_ENABLE,
    .entry_error_px = STEPMOTOR_Q4_RETURN_ENTRY_ERROR_PX,
    .exit_error_px = STEPMOTOR_Q4_RETURN_EXIT_ERROR_PX,
    .target_accel_px_s2 = STEPMOTOR_Q4_RETURN_ACCEL_PX_S2,
    .target_speed_max_px_s = STEPMOTOR_Q4_RETURN_SPEED_MAX_PX_S,
    .angle_kp = STEPMOTOR_Q4_RETURN_ANGLE_KP,
    .angle_kv = STEPMOTOR_Q4_RETURN_ANGLE_KV,
    .angle_limit_deg = STEPMOTOR_Q4_RETURN_ANGLE_LIMIT_DEG,
    .velocity_lpf_alpha = STEPMOTOR_Q4_RETURN_VELOCITY_LPF,
};

static PID_t BallPositionPID = {
    .Kp = STEPMOTOR_PID_KP,
    .Ki = STEPMOTOR_PID_KI,
    .Kd = STEPMOTOR_PID_KD,
    .Out_Max = STEPMOTOR_TUBE_ANGLE_MAX_DEG,
    .Out_Min = STEPMOTOR_TUBE_ANGLE_MIN_DEG,
    .integ_limit = STEPMOTOR_TUBE_ANGLE_MAX_DEG,
    .Out_Offset = 0.0f,
};

static volatile StepMotorControlState_t motor_state = STEPMOTOR_STATE_UNINITIALIZED;
static volatile StepMotorControlProfile_t active_profile =
    STEPMOTOR_PROFILE_DEFAULT;
static volatile bool profile_reset_pending;
static volatile uint16_t camera_center_x = STEPMOTOR_CAMERA_CENTER_X;
static volatile uint16_t latest_ball_x;
static volatile uint16_t target_ball_x;
static volatile uint32_t ball_sample_sequence;
static volatile bool angle_override_enabled;
static volatile float angle_override_deg;
static volatile bool angle_override_speed_enabled;
static volatile uint16_t angle_override_speed_rpm;
static float filtered_ball_x;
static bool ball_filter_initialized;
static bool ball_velocity_initialized;
static uint16_t ball_velocity_last_x;
static uint32_t ball_velocity_last_tick;
static float ball_velocity_px_s;
static bool question4_return_active;

static uint32_t processed_sample_sequence;
static uint32_t homing_start_tick;
static uint32_t zero_save_start_tick;
static float target_tube_angle_deg;
static float last_sent_tube_angle_deg;
static float ball_feedback_angle_deg;
static volatile float feedforward_acceleration_g;
static uint16_t last_sent_speed_rpm;
static bool target_angle_valid;
static bool command_sent;

static void StepMotor_ResetQuestion4Return(uint16_t ball_x);
static void StepMotor_UpdateBallVelocity(uint16_t ball_x);
static float StepMotor_Question4ReturnAngle(uint16_t ball_x,
                                            float pid_angle_deg);

#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
static uint16_t sweep_test_ball_x;
static uint32_t sweep_test_tick;
static bool sweep_test_running;
#endif

static StepMotorTune_t *StepMotor_GetActiveTune(void)
{
    if (active_profile == STEPMOTOR_PROFILE_QUESTION3) {
        return &StepMotorQuestion3Tune;
    }
    if (active_profile == STEPMOTOR_PROFILE_QUESTION3_NEGATIVE) {
        return &StepMotorQuestion3NegativeTune;
    }
    if (active_profile == STEPMOTOR_PROFILE_QUESTION4) {
        return &StepMotorQuestion4Tune;
    }
    return &StepMotorTune;
}

static void StepMotor_ResetControlAtZero(void)
{
    uint16_t center_x = StepMotor_GetCameraCenterX();

    PID_Clear(&BallPositionPID);
    target_tube_angle_deg = 0.0f;
    last_sent_tube_angle_deg = 0.0f;
    ball_feedback_angle_deg = 0.0f;
    target_ball_x = center_x;
    WaterTubeMotor.current_angle = 0.0f;
    WaterTubeMotor.target_angle = 0.0f;
    target_angle_valid = false;
    command_sent = false;
    last_sent_speed_rpm = 0U;
    angle_override_enabled = false;
    angle_override_deg = 0.0f;
    StepMotor_ResetQuestion4Return(latest_ball_x);
    processed_sample_sequence = ball_sample_sequence;
}

static float StepMotor_ClampAngle(float angle_deg)
{
    StepMotorTune_t *tune = StepMotor_GetActiveTune();
    float minimum = tune->angle_min_deg;
    float maximum = tune->angle_max_deg;

    if (minimum < STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG) {
        minimum = STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG;
    }
    if (maximum > STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) {
        maximum = STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG;
    }
    if (angle_deg > maximum) {
        return maximum;
    }
    if (angle_deg < minimum) {
        return minimum;
    }
    return angle_deg;
}

static float StepMotor_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float StepMotor_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float StepMotor_CopySign(float magnitude, float sign_source)
{
    return (sign_source < 0.0f) ? -magnitude : magnitude;
}

static void StepMotor_ResetBallVelocity(uint16_t ball_x)
{
    ball_velocity_last_x = ball_x;
    ball_velocity_last_tick = HAL_GetTick();
    ball_velocity_px_s = 0.0f;
    ball_velocity_initialized = true;
}

static void StepMotor_UpdateBallVelocity(uint16_t ball_x)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms;
    float raw_velocity_px_s;
    float alpha = StepMotorQuestion4ReturnTune.velocity_lpf_alpha;

    if (!ball_velocity_initialized) {
        StepMotor_ResetBallVelocity(ball_x);
        return;
    }

    elapsed_ms = now - ball_velocity_last_tick;
    if (elapsed_ms == 0U) {
        ball_velocity_last_x = ball_x;
        return;
    }

    raw_velocity_px_s = ((float)ball_x - (float)ball_velocity_last_x) *
                        1000.0f / (float)elapsed_ms;
    alpha = StepMotor_ClampFloat(alpha, 0.0f, 1.0f);
    ball_velocity_px_s += alpha * (raw_velocity_px_s - ball_velocity_px_s);
    ball_velocity_last_x = ball_x;
    ball_velocity_last_tick = now;
}

static void StepMotor_ResetQuestion4Return(uint16_t ball_x)
{
    question4_return_active = false;
    StepMotor_ResetBallVelocity(ball_x);
}

static float StepMotor_Question4ReturnAngle(uint16_t ball_x,
                                            float pid_angle_deg)
{
    StepMotorReturnTune_t *return_tune = &StepMotorQuestion4ReturnTune;
    float error_px;
    float abs_error_px;
    float entry_error_px;
    float exit_error_px;
    float target_accel_px_s2;
    float target_speed_px_s;
    float target_speed_max_px_s;
    float velocity_error_px_s;
    float return_angle_deg;
    float angle_limit_deg;

    if ((active_profile != STEPMOTOR_PROFILE_QUESTION4) ||
        (return_tune->enabled == 0U)) {
        question4_return_active = false;
        return pid_angle_deg;
    }

    error_px = (float)StepMotor_GetCameraCenterX() - (float)ball_x;
    abs_error_px = StepMotor_Abs(error_px);
    entry_error_px = StepMotor_Abs(return_tune->entry_error_px);
    exit_error_px = StepMotor_Abs(return_tune->exit_error_px);
    if (entry_error_px < 1.0f) {
        question4_return_active = false;
        return pid_angle_deg;
    }
    if (exit_error_px > entry_error_px) {
        exit_error_px = entry_error_px;
    }

    if (!question4_return_active && (abs_error_px >= entry_error_px)) {
        question4_return_active = true;
        PID_Clear(&BallPositionPID);
    }
    if (question4_return_active && (abs_error_px <= exit_error_px)) {
        question4_return_active = false;
        PID_Clear(&BallPositionPID);
        return pid_angle_deg;
    }
    if (!question4_return_active) {
        return pid_angle_deg;
    }

    target_accel_px_s2 = StepMotor_Abs(return_tune->target_accel_px_s2);
    target_speed_max_px_s = StepMotor_Abs(return_tune->target_speed_max_px_s);
    if ((target_accel_px_s2 < 1.0f) || (target_speed_max_px_s < 1.0f)) {
        return pid_angle_deg;
    }

    target_speed_px_s = sqrtf(2.0f * target_accel_px_s2 * abs_error_px);
    if (target_speed_px_s > target_speed_max_px_s) {
        target_speed_px_s = target_speed_max_px_s;
    }
    target_speed_px_s = StepMotor_CopySign(target_speed_px_s, error_px);
    velocity_error_px_s = target_speed_px_s - ball_velocity_px_s;

    return_angle_deg = (return_tune->angle_kp * error_px) +
                       (return_tune->angle_kv * velocity_error_px_s);
    angle_limit_deg = StepMotor_Abs(return_tune->angle_limit_deg);
    if (angle_limit_deg < 1.0f) {
        angle_limit_deg = 1.0f;
    } else if (angle_limit_deg > STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) {
        angle_limit_deg = STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG;
    }

    return StepMotor_ClampFloat(return_angle_deg,
                                -angle_limit_deg,
                                angle_limit_deg);
}

static void StepMotor_StoreBallX(uint16_t ball_x)
{
    if (ball_x > STEPMOTOR_CAMERA_X_MAX) {
        return;
    }

    latest_ball_x = ball_x;
    ball_sample_sequence++;
}

static void StepMotor_FilterAndStoreBallX(uint16_t ball_x)
{
    StepMotorTune_t *tune = StepMotor_GetActiveTune();
    float alpha;

    if (ball_x > STEPMOTOR_CAMERA_X_MAX) {
        return;
    }

    alpha = tune->ball_lpf_alpha;
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    if (!ball_filter_initialized) {
        filtered_ball_x = (float)ball_x;
        ball_filter_initialized = true;
    } else {
        filtered_ball_x += alpha * ((float)ball_x - filtered_ball_x);
    }

    StepMotor_StoreBallX((uint16_t)(filtered_ball_x + 0.5f));
}

#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
static void StepMotor_ResetSweepTest(void)
{
    sweep_test_ball_x = 0U;
    sweep_test_tick = HAL_GetTick() - STEPMOTOR_PIXEL_SWEEP_PERIOD_MS;
    sweep_test_running = true;
}

static void StepMotor_UpdateSweepTest(void)
{
    uint32_t now;

    if (!sweep_test_running) {
        return;
    }

    now = HAL_GetTick();
    if ((now - sweep_test_tick) < STEPMOTOR_PIXEL_SWEEP_PERIOD_MS) {
        return;
    }
    sweep_test_tick = now;
    StepMotor_StoreBallX(sweep_test_ball_x);
    if (sweep_test_ball_x < STEPMOTOR_CAMERA_X_MAX) {
        sweep_test_ball_x++;
    } else {
        sweep_test_running = false;
    }
}
#endif

static bool StepMotor_UartReady(void)
{
    return WaterTubeMotor.huart->gState == HAL_UART_STATE_READY;
}

static void StepMotor_SendTargetAngle(void)
{
    StepMotorTune_t *tune = StepMotor_GetActiveTune();
    uint16_t speed_rpm = tune->run_speed_rpm;
    float motor_angle_deg;
    uint32_t pulses;
    uint8_t direction;

    if (!target_angle_valid || !StepMotor_UartReady()) {
        return;
    }
    if (angle_override_enabled && angle_override_speed_enabled) {
        speed_rpm = angle_override_speed_rpm;
    }

    if (command_sent &&
        (last_sent_speed_rpm == speed_rpm) &&
        StepMotor_Abs(target_tube_angle_deg - last_sent_tube_angle_deg) <
            STEPMOTOR_COMMAND_DEADBAND_DEG) {
        return;
    }

    motor_angle_deg = target_tube_angle_deg * STEPMOTOR_MOTOR_DEG_PER_TUBE_DEG;
    direction = (motor_angle_deg >= 0.0f) ? STEPMOTOR_POSITIVE_DIRECTION :
                                           (uint8_t)!STEPMOTOR_POSITIVE_DIRECTION;
    pulses = (uint32_t)(StepMotor_Abs(motor_angle_deg) *
                        STEPMOTOR_PULSES_PER_REV / 360.0f + 0.5f);

    Emm_V5_Pos_Control(&WaterTubeMotor,
                       direction,
                       speed_rpm,
                       STEPMOTOR_RUN_ACCELERATION,
                       pulses,
                       STEPMOTOR_ABSOLUTE_POSITION_MODE,
                       false);

    WaterTubeMotor.current_angle = target_tube_angle_deg;
    last_sent_tube_angle_deg = target_tube_angle_deg;
    last_sent_speed_rpm = speed_rpm;
    command_sent = true;
}

void StepMotor_Init(void)
{
    uint16_t center_x = StepMotor_GetCameraCenterX();

    HAL_Delay(800);//上电等待电机稳定
    WaterTubeMotor.huart = &huart4;
    WaterTubeMotor.ID = STEPMOTOR_MOTOR_ID;
    WaterTubeMotor.current_angle = 0.0f;
    WaterTubeMotor.target_angle = 0.0f;
    WaterTubeMotor.target_speed = 0.0f;

    PID_Clear(&BallPositionPID);
    latest_ball_x = center_x;
    target_ball_x = center_x;
    filtered_ball_x = (float)center_x;
    ball_filter_initialized = false;
    ball_sample_sequence = 0U;
    processed_sample_sequence = 0U;
    target_tube_angle_deg = 0.0f;
    last_sent_tube_angle_deg = 0.0f;
    ball_feedback_angle_deg = 0.0f;
    feedforward_acceleration_g = 0.0f;
    target_angle_valid = false;
    command_sent = false;
    last_sent_speed_rpm = 0U;
    active_profile = STEPMOTOR_PROFILE_DEFAULT;
    profile_reset_pending = false;
    angle_override_enabled = false;
    angle_override_deg = 0.0f;
    angle_override_speed_enabled = false;
    angle_override_speed_rpm = 0U;
    ball_velocity_initialized = false;
    ball_velocity_last_x = center_x;
    ball_velocity_last_tick = HAL_GetTick();
    ball_velocity_px_s = 0.0f;
    question4_return_active = false;

    motor_state = STEPMOTOR_STATE_ENABLING;
    Emm_V5_En_Control(&WaterTubeMotor, true, false);
}

void StepMotor_SetBallX(uint16_t ball_x)
{
#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
    (void)ball_x;
#else
    StepMotor_FilterAndStoreBallX(ball_x);
#endif
}

bool StepMotor_SetCameraCenterX(uint16_t center_x)
{
    uint16_t old_center;

    if (center_x > STEPMOTOR_CAMERA_X_MAX) {
        return false;
    }

    old_center = camera_center_x;
    camera_center_x = center_x;
    if (target_ball_x == old_center) {
        target_ball_x = center_x;
    }
    return true;
}

uint16_t StepMotor_GetCameraCenterX(void)
{
    return camera_center_x;
}

bool StepMotor_SetBallTargetX(uint16_t target_x)
{
    if (target_x > STEPMOTOR_CAMERA_X_MAX) {
        return false;
    }

    target_ball_x = target_x;
    return true;
}

bool StepMotor_SetControlProfile(StepMotorControlProfile_t profile)
{
    if ((profile != STEPMOTOR_PROFILE_DEFAULT) &&
        (profile != STEPMOTOR_PROFILE_QUESTION3) &&
        (profile != STEPMOTOR_PROFILE_QUESTION3_NEGATIVE) &&
        (profile != STEPMOTOR_PROFILE_QUESTION4)) {
        return false;
    }

    if (active_profile != profile) {
        active_profile = profile;
        angle_override_enabled = false;
        angle_override_speed_enabled = false;
        profile_reset_pending = true;
    }
    return true;
}

void StepMotor_SetAngleOverride(bool enabled, float angle_deg)
{
    StepMotor_SetAngleOverrideWithSpeed(enabled, angle_deg, 0U);
}

void StepMotor_SetAngleOverrideWithSpeed(bool enabled,
                                         float angle_deg,
                                         uint16_t speed_rpm)
{
    if (enabled) {
        angle_override_deg = StepMotor_ClampAngle(angle_deg);
        angle_override_enabled = true;
        angle_override_speed_enabled = (speed_rpm != 0U) ? true : false;
        angle_override_speed_rpm = speed_rpm;
        command_sent = false;
        return;
    }

    if (angle_override_enabled) {
        angle_override_enabled = false;
        angle_override_speed_enabled = false;
        angle_override_speed_rpm = 0U;
        profile_reset_pending = true;
    }
}

void StepMotor_SetFeedforwardAcceleration(float acceleration_g)
{
    StepMotorTune_t *tune = StepMotor_GetActiveTune();
    float alpha = tune->feedforward_lpf_alpha;

    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    feedforward_acceleration_g +=
        alpha * (acceleration_g - feedforward_acceleration_g);
}

void StepMotor_SetTubeAngle(float tube_angle_deg)
{
    target_tube_angle_deg = StepMotor_ClampAngle(tube_angle_deg);
    WaterTubeMotor.target_angle = target_tube_angle_deg;
    target_angle_valid = true;
}

bool StepMotor_SetEnabled(bool enabled)
{
    if ((motor_state == STEPMOTOR_STATE_UNINITIALIZED) ||
        !StepMotor_UartReady()) {
        return false;
    }

    if (enabled) {
        if (motor_state == STEPMOTOR_STATE_READY) {
            return true;
        }
        if (motor_state != STEPMOTOR_STATE_DISABLED) {
            return false;
        }
        Emm_V5_En_Control(&WaterTubeMotor, true, false);
        motor_state = STEPMOTOR_STATE_ENABLING;
        return true;
    }

    if (motor_state == STEPMOTOR_STATE_DISABLED) {
        StepMotor_SetAngleOverride(false, 0.0f);
        return true;
    }
    if ((motor_state == STEPMOTOR_STATE_ZERO_SAVING) ||
        (motor_state == STEPMOTOR_STATE_ZERO_CLEARING)) {
        return false;
    }

    Emm_V5_En_Control(&WaterTubeMotor, false, false);
    StepMotor_SetAngleOverride(false, 0.0f);
    target_angle_valid = false;
    command_sent = false;
    last_sent_speed_rpm = 0U;
    motor_state = STEPMOTOR_STATE_DISABLED;
    return true;
}

bool StepMotor_SaveCurrentAsHome(void)
{
    if ((motor_state != STEPMOTOR_STATE_DISABLED) ||
        !StepMotor_UartReady()) {
        return false;
    }

    /* Keep the motor disabled while storing the hand-adjusted position. */
    Emm_V5_Origin_Set_O(&WaterTubeMotor, true);
    zero_save_start_tick = HAL_GetTick();
    motor_state = STEPMOTOR_STATE_ZERO_SAVING;
    return true;
}

void StepMotor_Task(void)
{
    StepMotorTune_t *tune;
    uint32_t sample_sequence;
    uint16_t ball_x;

    if (motor_state == STEPMOTOR_STATE_UNINITIALIZED) {
        return;
    }

    if (motor_state == STEPMOTOR_STATE_ENABLING) {
        if (!StepMotor_UartReady()) {
            return;
        }
        Emm_V5_Origin_Trigger_Return(&WaterTubeMotor,
                                     STEPMOTOR_HOME_MODE,
                                     false);
        homing_start_tick = HAL_GetTick();
        motor_state = STEPMOTOR_STATE_HOMING;
        return;
    }

    if (motor_state == STEPMOTOR_STATE_HOMING) {
        if ((HAL_GetTick() - homing_start_tick) < STEPMOTOR_HOME_WAIT_MS) {
            return;
        }

        StepMotor_ResetControlAtZero();
#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
        StepMotor_ResetSweepTest();
#endif
        motor_state = STEPMOTOR_STATE_READY;
        return;
    }

    if (motor_state == STEPMOTOR_STATE_ZERO_SAVING) {
        if (!StepMotor_UartReady() ||
            ((HAL_GetTick() - zero_save_start_tick) <
             STEPMOTOR_ZERO_SAVE_WAIT_MS)) {
            return;
        }
        Emm_V5_Reset_CurPos_To_Zero(&WaterTubeMotor);
        motor_state = STEPMOTOR_STATE_ZERO_CLEARING;
        return;
    }

    if (motor_state == STEPMOTOR_STATE_ZERO_CLEARING) {
        if (!StepMotor_UartReady()) {
            return;
        }
        StepMotor_ResetControlAtZero();
        motor_state = STEPMOTOR_STATE_DISABLED;
        return;
    }

    if (motor_state != STEPMOTOR_STATE_READY) {
        return;
    }

    if (profile_reset_pending) {
        profile_reset_pending = false;
        PID_Clear(&BallPositionPID);
        ball_feedback_angle_deg = 0.0f;
        filtered_ball_x = (float)latest_ball_x;
        ball_filter_initialized = true;
        feedforward_acceleration_g = 0.0f;
        StepMotor_ResetQuestion4Return(latest_ball_x);
        command_sent = false;
        last_sent_speed_rpm = 0U;
        processed_sample_sequence = ball_sample_sequence - 1U;
    }

    tune = StepMotor_GetActiveTune();

    if (angle_override_enabled) {
        StepMotor_SetTubeAngle(angle_override_deg);
        StepMotor_SendTargetAngle();
        return;
    }

#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
    StepMotor_UpdateSweepTest();
#endif

    sample_sequence = ball_sample_sequence;
    if (sample_sequence != processed_sample_sequence) {
        ball_x = latest_ball_x;
        processed_sample_sequence = sample_sequence;
        StepMotor_UpdateBallVelocity(ball_x);

        BallPositionPID.Kp = tune->ball_kp;
        BallPositionPID.Ki = tune->ball_ki;
        BallPositionPID.Kd = tune->ball_kd;
        BallPositionPID.Out_Min = tune->angle_min_deg;
        BallPositionPID.Out_Max = tune->angle_max_deg;
        BallPositionPID.integ_limit =
            (StepMotor_Abs(tune->angle_min_deg) > tune->angle_max_deg) ?
            StepMotor_Abs(tune->angle_min_deg) : tune->angle_max_deg;
        BallPositionPID.target = (float)target_ball_x;
        BallPositionPID.actual = (float)ball_x;
        PID_Calc(&BallPositionPID);
        ball_feedback_angle_deg = StepMotor_Question4ReturnAngle(
            ball_x, BallPositionPID.output);
    }

#if STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE
    StepMotor_SetTubeAngle(ball_feedback_angle_deg);
#else
    StepMotor_SetTubeAngle(ball_feedback_angle_deg +
                           tune->angle_bias_deg +
                           tune->ball_kff *
                               feedforward_acceleration_g);
#endif

    StepMotor_SendTargetAngle();
}

StepMotorControlState_t StepMotor_GetState(void)
{
    return motor_state;
}

float StepMotor_GetTargetAngle(void)
{
    return target_tube_angle_deg;
}

uint16_t StepMotor_GetBallX(void)
{
    return latest_ball_x;
}

uint16_t StepMotor_GetBallTargetX(void)
{
    return target_ball_x;
}

StepMotorControlProfile_t StepMotor_GetControlProfile(void)
{
    return active_profile;
}
