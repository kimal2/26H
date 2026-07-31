#ifndef TUBE_TASK_H
#define TUBE_TASK_H

#include "stm32f4xx_hal.h"

void StartTubeTask(void *argument);
void TubeTask_Process(void);

#endif
