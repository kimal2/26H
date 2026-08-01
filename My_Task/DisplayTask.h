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

#define QUESTION4_ODOMETER_TARGET_DEFAULT  0ULL

void StartDisplayTask(void *argument);
void DisplayTask_Process(void);
void DisplayTask_ReportTrackingFinished(void);
void DisplayTask_SetQuestion3BrakeTriggerX(uint16_t trigger_x);
void DisplayTask_SetQuestion3BrakeAngle(float angle_deg);
void DisplayTask_SetQuestion3BrakeDuration(uint32_t duration_ms);
void DisplayTask_SetQuestion4OdometerTarget(uint64_t target_counts);
uint64_t DisplayTask_GetQuestion4OdometerTarget(void);

#endif
