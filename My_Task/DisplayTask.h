#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "stm32f4xx_hal.h"

#define QUESTION3_STRATEGY_BRAKE_PULSE  1U
#define QUESTION3_STRATEGY_DUAL_PID     2U

#ifndef QUESTION3_CONTROL_STRATEGY
#define QUESTION3_CONTROL_STRATEGY      QUESTION3_STRATEGY_BRAKE_PULSE
#endif

#if (QUESTION3_CONTROL_STRATEGY != QUESTION3_STRATEGY_BRAKE_PULSE) && \
    (QUESTION3_CONTROL_STRATEGY != QUESTION3_STRATEGY_DUAL_PID)
#error "Invalid QUESTION3_CONTROL_STRATEGY"
#endif

#define QUESTION3_BRAKE_TRIGGER_X       355U
#define QUESTION3_BRAKE_ANGLE_DEG       30.0f
#define QUESTION3_BRAKE_DURATION_MS     390U
#define QUESTION3_BRAKE_RPM             150U

#define QUESTION2_ODOMETER_TARGET_DEFAULT  42075ULL
#define QUESTION4_ODOMETER_TARGET_DEFAULT  14000ULL
#define QUESTION4_DRIVE_PWM_DEFAULT         300.0f
#define QUESTION4_DRIVE_ACCEL_DEFAULT       0.0f
#define QUESTION4_DRIVE_ACCEL_MAX           10000.0f

void StartDisplayTask(void *argument);
void DisplayTask_Process(void);
void DisplayTask_ReportTrackingFinished(void);
void DisplayTask_SetQuestion3BrakeTriggerX(uint16_t trigger_x);
void DisplayTask_SetQuestion3BrakeAngle(float angle_deg);
void DisplayTask_SetQuestion3BrakeDuration(uint32_t duration_ms);
void DisplayTask_SetQuestion3BrakeSpeed(uint16_t speed_rpm);
uint16_t DisplayTask_GetQuestion3BrakeTriggerX(void);
float DisplayTask_GetQuestion3BrakeAngle(void);
uint32_t DisplayTask_GetQuestion3BrakeDuration(void);
uint16_t DisplayTask_GetQuestion3BrakeSpeed(void);
void DisplayTask_SetQuestion2OdometerTarget(uint64_t target_counts);
uint64_t DisplayTask_GetQuestion2OdometerTarget(void);
void DisplayTask_SetQuestion4DrivePWM(float drive_pwm);
void DisplayTask_SetQuestion4DriveAcceleration(float pwm_per_second);
float DisplayTask_GetQuestion4DrivePWM(void);
float DisplayTask_GetQuestion4DriveAcceleration(void);
void DisplayTask_SetQuestion4OdometerTarget(uint64_t target_counts);
uint64_t DisplayTask_GetQuestion4OdometerTarget(void);

#endif
