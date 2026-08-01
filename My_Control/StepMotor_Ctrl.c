#include "StepMotor_Ctrl.h"

#include "pid.h"
#include "usart.h"

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
static volatile uint16_t latest_ball_x;
static volatile uint16_t target_ball_x;
static volatile uint32_t ball_sample_sequence;
static volatile bool angle_override_enabled;
static volatile float angle_override_deg;
static float filtered_ball_x;
static bool ball_filter_initialized;

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
    PID_Clear(&BallPositionPID);
    target_tube_angle_deg = 0.0f;
    last_sent_tube_angle_deg = 0.0f;
    ball_feedback_angle_deg = 0.0f;
    target_ball_x = STEPMOTOR_CAMERA_CENTER_X;
    WaterTubeMotor.current_angle = 0.0f;
    WaterTubeMotor.target_angle = 0.0f;
    target_angle_valid = false;
    command_sent = false;
    last_sent_speed_rpm = 0U;
    angle_override_enabled = false;
    angle_override_deg = 0.0f;
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
    float motor_angle_deg;
    uint32_t pulses;
    uint8_t direction;

    if (!target_angle_valid || !StepMotor_UartReady()) {
        return;
    }

    if (command_sent &&
        (last_sent_speed_rpm == tune->run_speed_rpm) &&
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
                       tune->run_speed_rpm,
                       STEPMOTOR_RUN_ACCELERATION,
                       pulses,
                       STEPMOTOR_ABSOLUTE_POSITION_MODE,
                       false);

    WaterTubeMotor.current_angle = target_tube_angle_deg;
    last_sent_tube_angle_deg = target_tube_angle_deg;
    last_sent_speed_rpm = tune->run_speed_rpm;
    command_sent = true;
}

void StepMotor_Init(void)
{
    HAL_Delay(800);//上电等待电机稳定
    WaterTubeMotor.huart = &huart4;
    WaterTubeMotor.ID = STEPMOTOR_MOTOR_ID;
    WaterTubeMotor.current_angle = 0.0f;
    WaterTubeMotor.target_angle = 0.0f;
    WaterTubeMotor.target_speed = 0.0f;

    PID_Clear(&BallPositionPID);
    latest_ball_x = STEPMOTOR_CAMERA_CENTER_X;
    target_ball_x = STEPMOTOR_CAMERA_CENTER_X;
    filtered_ball_x = (float)STEPMOTOR_CAMERA_CENTER_X;
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
        profile_reset_pending = true;
    }
    return true;
}

void StepMotor_SetAngleOverride(bool enabled, float angle_deg)
{
    if (enabled) {
        angle_override_deg = StepMotor_ClampAngle(angle_deg);
        angle_override_enabled = true;
        command_sent = false;
        return;
    }

    if (angle_override_enabled) {
        angle_override_enabled = false;
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
        ball_feedback_angle_deg = BallPositionPID.output;
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
