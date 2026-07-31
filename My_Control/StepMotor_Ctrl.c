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
static volatile uint16_t latest_ball_x;
static volatile uint32_t ball_sample_sequence;

static uint32_t processed_sample_sequence;
static uint32_t homing_start_tick;
static float target_tube_angle_deg;
static float last_sent_tube_angle_deg;
static float ball_feedback_angle_deg;
static volatile float feedforward_acceleration_g;
static bool target_angle_valid;
static bool command_sent;

static float StepMotor_ClampAngle(float angle_deg)
{
    if (angle_deg > STEPMOTOR_TUBE_ANGLE_MAX_DEG) {
        return STEPMOTOR_TUBE_ANGLE_MAX_DEG;
    }
    if (angle_deg < STEPMOTOR_TUBE_ANGLE_MIN_DEG) {
        return STEPMOTOR_TUBE_ANGLE_MIN_DEG;
    }
    return angle_deg;
}

static float StepMotor_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool StepMotor_UartReady(void)
{
    return WaterTubeMotor.huart->gState == HAL_UART_STATE_READY;
}

static void StepMotor_SendTargetAngle(void)
{
    float motor_angle_deg;
    uint32_t pulses;
    uint8_t direction;

    if (!target_angle_valid || !StepMotor_UartReady()) {
        return;
    }

    if (command_sent &&
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
                       STEPMOTOR_RUN_SPEED_RPM,
                       STEPMOTOR_RUN_ACCELERATION,
                       pulses,
                       STEPMOTOR_ABSOLUTE_POSITION_MODE,
                       false);

    WaterTubeMotor.current_angle = target_tube_angle_deg;
    last_sent_tube_angle_deg = target_tube_angle_deg;
    command_sent = true;
}

void StepMotor_Init(void)
{
    WaterTubeMotor.huart = &huart4;
    WaterTubeMotor.ID = STEPMOTOR_MOTOR_ID;
    WaterTubeMotor.current_angle = 0.0f;
    WaterTubeMotor.target_angle = 0.0f;
    WaterTubeMotor.target_speed = 0.0f;

    PID_Clear(&BallPositionPID);
    latest_ball_x = STEPMOTOR_CAMERA_CENTER_X;
    ball_sample_sequence = 0U;
    processed_sample_sequence = 0U;
    target_tube_angle_deg = 0.0f;
    last_sent_tube_angle_deg = 0.0f;
    ball_feedback_angle_deg = 0.0f;
    feedforward_acceleration_g = 0.0f;
    target_angle_valid = false;
    command_sent = false;

    motor_state = STEPMOTOR_STATE_HOMING;
    Emm_V5_Origin_Trigger_Return(&WaterTubeMotor,
                                 STEPMOTOR_HOME_MODE,
                                 false);
    homing_start_tick = HAL_GetTick();
}

void StepMotor_SetBallX(uint16_t ball_x)
{
    if (ball_x > STEPMOTOR_CAMERA_X_MAX) {
        return;
    }

    latest_ball_x = ball_x;
    ball_sample_sequence++;
}

void StepMotor_SetFeedforwardAcceleration(float acceleration_g)
{
    feedforward_acceleration_g = acceleration_g;
}

void StepMotor_SetTubeAngle(float tube_angle_deg)
{
    target_tube_angle_deg = StepMotor_ClampAngle(tube_angle_deg);
    WaterTubeMotor.target_angle = target_tube_angle_deg;
    target_angle_valid = true;
}

void StepMotor_Task(void)
{
    uint32_t sample_sequence;
    uint16_t ball_x;

    if (motor_state == STEPMOTOR_STATE_UNINITIALIZED) {
        return;
    }

    if (motor_state == STEPMOTOR_STATE_HOMING) {
        if ((HAL_GetTick() - homing_start_tick) < STEPMOTOR_HOME_WAIT_MS) {
            return;
        }

        motor_state = STEPMOTOR_STATE_READY;
        return;
    }

    sample_sequence = ball_sample_sequence;
    if (sample_sequence != processed_sample_sequence) {
        ball_x = latest_ball_x;
        processed_sample_sequence = sample_sequence;

        BallPositionPID.Kp = StepMotorTune.ball_kp;
        BallPositionPID.Ki = StepMotorTune.ball_ki;
        BallPositionPID.Kd = StepMotorTune.ball_kd;
        BallPositionPID.target = (float)STEPMOTOR_CAMERA_CENTER_X;
        BallPositionPID.actual = (float)ball_x;
        PID_Calc(&BallPositionPID);
        ball_feedback_angle_deg = BallPositionPID.output;
    }

    StepMotor_SetTubeAngle(ball_feedback_angle_deg +
                           StepMotorTune.ball_kff *
                               feedforward_acceleration_g);

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
