#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

typedef enum {
    LINE_TYPE_UNKNOWN = 0,
    LINE_TYPE_STRAIGHT,
    LINE_TYPE_CROSS,
    LINE_TYPE_LEFT,
    LINE_TYPE_RIGHT,
    LINE_TYPE_LOST
} LineType;

uint8_t LineSensor_Init(void);
void LineSensor_Scan(void);
LineType LineService_Get_LineType(void);
int8_t LineSensor_Get_Err(void);
uint8_t LineSensor_GetBit(void);
uint8_t LineSensor_Get_BlackNum(void);

#endif
