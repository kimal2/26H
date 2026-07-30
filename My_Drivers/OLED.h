#ifndef __OLED_H
#define __OLED_H

#include "soft_i2c.h"

void OLED_Init(void);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey);
void OLED_Printf(uint8_t x, uint8_t y, uint8_t size,
                 const char *format, ...);
void OLED_Clear(void);


#endif
