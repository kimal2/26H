#ifndef GRAY_TRACK_TASK_H
#define GRAY_TRACK_TASK_H

#include "stm32f4xx_hal.h"

void StartGrayTrackTask(void *argument);
void GrayTrackTask_Process(void);

#endif
