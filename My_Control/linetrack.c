#include "linetrack.h"

#include "LineSensor.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

#define TRACKING_LINE_INTEGRAL_LIMIT  100.0f

Tracker_t Tracker;
TrackingTune_t TrackingTune = {
    .line_kp = TRACKING_KP_DEFAULT,
    .line_ki = TRACKING_KI_DEFAULT,
    .line_kd = TRACKING_KD_DEFAULT,
    .base_pwm = TRACKING_BASE_PWM_DEFAULT,
    .turn_pwm = TRACKING_TURN_PWM_DEFAULT,
    .max_correction = TRACKING_MAX_CORRECTION_DEFAULT,
};

static uint8_t tracking_ready;
static uint32_t tracking_start_tick;
static LineType last_branch = LINE_TYPE_UNKNOWN;
static int8_t last_line_error;
static float line_error_integral;

static float Tracking_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void Tracking_DriveLine(void)
{
    int8_t error;
    float correction;
    float left_pwm;
    float right_pwm;

    error = LineSensor_Get_Err();
    line_error_integral = Tracking_Clamp(
        line_error_integral + (float)error,
        -TRACKING_LINE_INTEGRAL_LIMIT,
        TRACKING_LINE_INTEGRAL_LIMIT);
    correction = TrackingTune.line_kp * (float)error +
                 TrackingTune.line_ki * line_error_integral +
                 TrackingTune.line_kd *
                     ((float)error - (float)last_line_error);
    correction = Tracking_Clamp(correction,
                                -TrackingTune.max_correction,
                                TrackingTune.max_correction);

    left_pwm = TrackingTune.base_pwm + correction;
    right_pwm = TrackingTune.base_pwm - correction;
    Motor_Set_PWM((int32_t)left_pwm, (int32_t)right_pwm);
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
        last_line_error = LineSensor_Get_Err();
        return TRACK_EVENT_LINE_REACQUIRED;
    }

    return TRACK_EVENT_NONE;
}

void Tracking_Init(void)
{
    Motor_Init();
    Motor_Stop();

    Tracker.car_state = CAR_STATE_STOP;
    tracking_start_tick = 0U;
    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    line_error_integral = 0.0f;
    tracking_ready = 1U;
}

void Tracking_Start(void)
{
    if (tracking_ready == 0U) {
        return;
    }

    tracking_start_tick = HAL_GetTick();
    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    line_error_integral = 0.0f;
    Tracker.car_state = CAR_STATE_TRACKING;
    Motor_Set_PWM((int32_t)TrackingTune.base_pwm,
                  (int32_t)TrackingTune.base_pwm);
}

void Tracking_Stop(void)
{
    Tracker.car_state = CAR_STATE_STOP;
    last_branch = LINE_TYPE_UNKNOWN;
    last_line_error = 0;
    line_error_integral = 0.0f;
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

TrackingEvent Tracking_Update(void)
{
    LineType line;
    TrackingEvent event;

    LineSensor_Scan();
    if (Tracker.car_state == CAR_STATE_STOP) {
        return TRACK_EVENT_NONE;
    }

    if (((uint32_t)(HAL_GetTick() - tracking_start_tick) >=
         TRACKING_STOP_LINE_DELAY_MS) &&
        (LineSensor_Get_BlackNum() >= TRACKING_STOP_LINE_BLACK_MIN)) {
        return TRACK_EVENT_FINISH_LINE;
    }

    line = LineService_Get_LineType();
    event = Tracking_UpdateState(line);

    if (Tracker.car_state == CAR_STATE_TURN_LEFT) {
        Motor_Set_PWM(-(int32_t)TrackingTune.turn_pwm,
                      (int32_t)TrackingTune.turn_pwm);
    } else if (Tracker.car_state == CAR_STATE_TURN_RIGHT) {
        Motor_Set_PWM((int32_t)TrackingTune.turn_pwm,
                      -(int32_t)TrackingTune.turn_pwm);
    } else if (line == LINE_TYPE_LOST) {
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
