#ifndef LINETRACK_H
#define LINETRACK_H

#include <stdint.h>

#define TRACKING_PWM_MAX                1000.0f
#define TRACKING_BASE_PWM_DEFAULT       300.0f
#define TRACKING_TURN_PWM_DEFAULT       200.0f
#define TRACKING_KP_DEFAULT             10.0f
#define TRACKING_KI_DEFAULT             0.0f
#define TRACKING_KD_DEFAULT             0.0f
#define TRACKING_MAX_CORRECTION_DEFAULT 350.0f
#define TRACKING_ODOMETER_TARGET_DEFAULT 42075ULL

typedef struct {
    volatile float line_kp;
    volatile float line_ki;
    volatile float line_kd;
    volatile float base_pwm;
    volatile float turn_pwm;
    volatile float max_correction;
} TrackingTune_t;

typedef enum {
    CAR_STATE_TRACKING = 0,
    CAR_STATE_TURN_LEFT,
    CAR_STATE_TURN_RIGHT,
    CAR_STATE_STOP
} Car_State;

typedef enum {
    TRACK_EVENT_NONE = 0,
    TRACK_EVENT_TURN_LEFT,
    TRACK_EVENT_TURN_RIGHT,
    TRACK_EVENT_LINE_REACQUIRED,
    TRACK_EVENT_FINISH_LINE
} TrackingEvent;

typedef struct {
    volatile Car_State car_state;
} Tracker_t;

extern TrackingTune_t TrackingTune;

void Tracking_Init(void);
void Tracking_Start(void);
void Tracking_Stop(void);
void Tracking_Toggle(void);
TrackingEvent Tracking_Update(void);
Car_State Tracking_GetState(void);
uint8_t Tracking_IsReady(void);
int8_t Tracking_GetLineError(void);
uint8_t Tracking_GetBlackCount(void);
float Tracking_GetDiffPWM(void);
void Tracking_SetOdometerTarget(uint64_t target_counts);
uint64_t Tracking_GetOdometerTarget(void);
uint64_t Tracking_GetOdometerCounts(void);

#endif
