#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

void Motor_Init(void);
void Motor_Set_PWM(int32_t PWM_L, int32_t PWM_R);
void Motor_Stop(void);
void Motor_Sleep(void);

#endif
