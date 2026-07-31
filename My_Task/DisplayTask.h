#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "stm32f4xx_hal.h"

void StartDisplayTask(void *argument);
void DisplayTask_Process(void);
void DisplayTask_ReportTrackingFinished(void);

#endif
