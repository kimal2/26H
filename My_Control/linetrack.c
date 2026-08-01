#include "linetrack.h"

#include "LineSensor.h"
#include "motor.h"
#include "pid.h"
#include "stm32f4xx_hal.h"

#define TRACKING_LINE_INTEGRAL_LIMIT  100.0f

Tracker_t Tracker;
TrackingTune_t TrackingQuestion2Tune = {
    .line_kp = TRACKING_KP_DEFAULT,
    .line_ki = TRACKING_KI_DEFAULT,
    .line_kd = TRACKING_KD_DEFAULT,
    .base_pwm = TRACKING_BASE_PWM_DEFAULT,
    .turn_pwm = TRACKING_TURN_PWM_DEFAULT,
    .max_correction = TRACKING_MAX_CORRECTION_DEFAULT,
};
TrackingTune_t TrackingQuestion4Tune = {
    .line_kp = TRACKING_KP_DEFAULT,
    .line_ki = TRACKING_KI_DEFAULT,
    .line_kd = TRACKING_KD_DEFAULT,
    .base_pwm = TRACKING_BASE_PWM_DEFAULT,
    .turn_pwm = TRACKING_TURN_PWM_DEFAULT,
    .max_correction = TRACKING_MAX_CORRECTION_DEFAULT,
};

static uint8_t tracking_ready;
static TrackingTune_t *active_tracking_tune = &TrackingQuestion2Tune;
static LineType last_branch = LINE_TYPE_UNKNOWN;
static int8_t last_line_error;
static volatile int8_t current_line_error;
static volatile uint8_t current_black_count;
static volatile float current_diff_pwm;
static volatile float drive_acceleration_pwm_per_second;
static volatile uint64_t odometer_target_counts =
    TRACKING_ODOMETER_TARGET_DEFAULT;
static float current_left_pwm;
static float current_right_pwm;
static uint32_t drive_command_tick;

static PID_t LineTrackingPID = {
    .Kp = TRACKING_KP_DEFAULT,
    .Ki = TRACKING_KI_DEFAULT,
    .Kd = TRACKING_KD_DEFAULT,
    .Out_Max = TRACKING_MAX_CORRECTION_DEFAULT,
    .Out_Min = -TRACKING_MAX_CORRECTION_DEFAULT,
    .integ_limit = TRACKING_LINE_INTEGRAL_LIMIT,
    .Out_Offset = 0.0f,
};

static void Tracking_ResetPID(int8_t error)
{
    PID_Clear(&LineTrackingPID);
    LineTrackingPID.actual = (float)error;
    LineTrackingPID.actual_last = (float)error;
    current_diff_pwm = 0.0f;
}

static float Tracking_SlewPWM(float current, float target, float maximum_delta)
{
    if (target > (current + maximum_delta)) {
        return current + maximum_delta;
    }
    if (target < (current - maximum_delta)) {
        return current - maximum_delta;
    }
    return target;
}

static void Tracking_ResetDriveCommand(void)
{
    current_left_pwm = 0.0f;
    current_right_pwm = 0.0f;
    drive_command_tick = HAL_GetTick();
}

static void Tracking_ApplyDrivePWM(float left_pwm, float right_pwm)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - drive_command_tick;
    float acceleration = drive_acceleration_pwm_per_second;

    drive_command_tick = now;
    if (acceleration > 0.0f) {
        float maximum_delta = acceleration * (float)elapsed_ms / 1000.0f;

        current_left_pwm = Tracking_SlewPWM(
            current_left_pwm, left_pwm, maximum_delta);
        current_right_pwm = Tracking_SlewPWM(
            current_right_pwm, right_pwm, maximum_delta);
    } else {
        current_left_pwm = left_pwm;
        current_right_pwm = right_pwm;
    }

    Motor_Set_PWM((int32_t)current_left_pwm, (int32_t)current_right_pwm);
}

static void Tracking_DriveLine(void)
{
    int8_t error;
    float avg_pwm;
    float diff_pwm;
    float left_pwm;
    float right_pwm;

    error = current_line_error;
    LineTrackingPID.Kp = active_tracking_tune->line_kp;
    LineTrackingPID.Ki = active_tracking_tune->line_ki;
    LineTrackingPID.Kd = active_tracking_tune->line_kd;
    LineTrackingPID.Out_Max = active_tracking_tune->max_correction;
    LineTrackingPID.Out_Min = -active_tracking_tune->max_correction;
    LineTrackingPID.target = 0.0f;
    LineTrackingPID.actual = (float)error;
    PID_Calc(&LineTrackingPID);

    avg_pwm = active_tracking_tune->base_pwm;
    diff_pwm = LineTrackingPID.output;
    left_pwm = avg_pwm - diff_pwm;
    right_pwm = avg_pwm + diff_pwm;
    Tracking_ApplyDrivePWM(left_pwm, right_pwm);
    current_diff_pwm = diff_pwm;
    last_line_error = error;
}

static TrackingEvent Tracking_UpdateState(LineType line)
{
    if (line == LINE_TYPE_LEFT) {
        last_branch = LINE_TYPE_LEFT;
    } else if (line == LINE_TYPE_RIGHT) {
        last_branch = LINE_TYPE_RIGHT;
    }

    if (Tracker.car_state == CAR_STATE_TRACKING) {
        if (line != LINE_TYPE_LOST) {
            return TRACK_EVENT_NONE;
        }

        if ((last_branch == LINE_TYPE_LEFT) ||
            ((last_branch == LINE_TYPE_UNKNOWN) && (last_line_error < 0))) {
            Tracker.car_state = CAR_STATE_TURN_LEFT;
            return TRACK_EVENT_TURN_LEFT;
        }
        if ((last_branch == LINE_TYPE_RIGHT) ||
            ((last_branch == LINE_TYPE_UNKNOWN) && (last_line_error > 0))) {
            Tracker.car_state = CAR_STATE_TURN_RIGHT;
            return TRACK_EVENT_TURN_RIGHT;
        }
        return TRACK_EVENT_NONE;
    }

    if (((Tracker.car_state == CAR_STATE_TURN_LEFT) ||
         (Tracker.car_state == CAR_STATE_TURN_RIGHT)) &&
        (line == LINE_TYPE_STRAIGHT)) {
        Tracker.car_state = CAR_STATE_TRACKING;
        last_branch = LINE_TYPE_UNKNOWN;
        last_line_error = current_line_error;
        Tracking_ResetPID(current_line_error);
        return TRACK_EVENT_LINE_REACQUIRED;
    }

    return TRACK_EVENT_NONE;
}

void Tracking_Init(void)
{
    Motor_Init();
    Motor_Stop();

    Tracker.car_state = CAR_STATE_STOP;
    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    current_line_error = 0;
    current_black_count = 0U;
    Tracking_ResetPID(0);
    Tracking_ResetDriveCommand();
    tracking_ready = 1U;
}

void Tracking_Start(void)
{
    if (tracking_ready == 0U) {
        return;
    }

    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    Tracking_ResetPID(current_line_error);
    Motor_OdometerReset();
    Tracking_ResetDriveCommand();
    Tracker.car_state = CAR_STATE_TRACKING;
    Tracking_ApplyDrivePWM(active_tracking_tune->base_pwm,
                           active_tracking_tune->base_pwm);
}

void Tracking_Stop(void)
{
    Tracker.car_state = CAR_STATE_STOP;
    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    Tracking_ResetPID(current_line_error);
    Tracking_ResetDriveCommand();
    Motor_Stop();
}

void Tracking_Toggle(void)
{
    if (Tracker.car_state == CAR_STATE_STOP) {
        Tracking_Start();
    } else {
        Tracking_Stop();
    }
}

void Tracking_SetTune(TrackingTune_t *tune)
{
    active_tracking_tune = (tune != 0) ? tune : &TrackingQuestion2Tune;
    Tracking_ResetPID(current_line_error);
}

TrackingTune_t *Tracking_GetTune(void)
{
    return active_tracking_tune;
}

TrackingEvent Tracking_Update(void)
{
    LineType line;
    TrackingEvent event;
    uint64_t odometer_target;

    LineSensor_Scan();
    Motor_EncoderUpdate();
    current_line_error = LineSensor_Get_Err();
    current_black_count = LineSensor_Get_BlackNum();
    if (Tracker.car_state == CAR_STATE_STOP) {
        return TRACK_EVENT_NONE;
    }

    odometer_target = Tracking_GetOdometerTarget();
    if ((odometer_target != 0U) &&
        (Motor_GetOdometerCounts() >= odometer_target)) {
        return TRACK_EVENT_FINISH_LINE;
    }

    line = LineService_Get_LineType();
    event = Tracking_UpdateState(line);

    if (Tracker.car_state == CAR_STATE_TURN_LEFT) {
        current_diff_pwm = 0.0f;
        Tracking_ApplyDrivePWM(-active_tracking_tune->turn_pwm,
                              active_tracking_tune->turn_pwm);
    } else if (Tracker.car_state == CAR_STATE_TURN_RIGHT) {
        current_diff_pwm = 0.0f;
        Tracking_ApplyDrivePWM(active_tracking_tune->turn_pwm,
                              -active_tracking_tune->turn_pwm);
    } else if (line == LINE_TYPE_LOST) {
        current_diff_pwm = 0.0f;
        Tracking_ResetDriveCommand();
        Motor_Stop();
    } else {
        Tracking_DriveLine();
    }

    return event;
}

Car_State Tracking_GetState(void)
{
    return Tracker.car_state;
}

uint8_t Tracking_IsReady(void)
{
    return tracking_ready;
}

int8_t Tracking_GetLineError(void)
{
    return current_line_error;
}

uint8_t Tracking_GetBlackCount(void)
{
    return current_black_count;
}

float Tracking_GetDiffPWM(void)
{
    return current_diff_pwm;
}

void Tracking_SetDriveAcceleration(float pwm_per_second)
{
    drive_acceleration_pwm_per_second =
        (pwm_per_second > 0.0f) ? pwm_per_second : 0.0f;
}

float Tracking_GetDriveAcceleration(void)
{
    return drive_acceleration_pwm_per_second;
}

void Tracking_SetOdometerTarget(uint64_t target_counts)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    odometer_target_counts = target_counts;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint64_t Tracking_GetOdometerTarget(void)
{
    uint64_t target;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    target = odometer_target_counts;
    if (primask == 0U) {
        __enable_irq();
    }
    return target;
}

uint64_t Tracking_GetOdometerCounts(void)
{
    return Motor_GetOdometerCounts();
}
