#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

void Encoder_Init(void);
int32_t Encoder_Get(uint8_t L_R);
int32_t Encoder_Get_L(void);
int32_t Encoder_Get_R(void);
void Encoder_Clear(void);

#endif
