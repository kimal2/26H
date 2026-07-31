#include "LineSensor.h"

#include "graysensor.h"

static const int8_t line_error_table[8] = {
    -20, -10, -5, -1, 1, 5, 10, 20
};

static uint8_t LineSensor_CountBlack(uint8_t bits)
{
    uint8_t count;
    uint8_t index;

    count = 0U;
    for (index = 0U; index < 8U; index++) {
        if ((bits & (uint8_t)(1U << index)) == 0U) {
            count++;
        }
    }
    return count;
}

void LineSensor_Init(void)
{
    (void)GraySensor_Init(&gray_sensor);
}

void LineSensor_Scan(void)
{
    (void)GraySensor_GetDigital(&gray_sensor);
}

int8_t LineSensor_Get_Err(void)
{
    int8_t sum;
    uint8_t bits;
    uint8_t index;

    sum = 0;
    bits = gray_sensor.digital;
    for (index = 0U; index < 8U; index++) {
        if ((bits & (uint8_t)(0x80U >> index)) == 0U) {
            sum += line_error_table[index];
        }
    }
    return sum;
}

uint8_t LineSensor_Get_BlackNum(void)
{
    return LineSensor_CountBlack(gray_sensor.digital);
}

LineType LineService_Get_LineType(void)
{
    uint8_t bits;
    uint8_t black;

    bits = gray_sensor.digital;
    black = LineSensor_CountBlack(bits);

    if (bits == 0xFFU) {
        return LINE_TYPE_LOST;
    }
    if (bits == 0x00U) {
        return LINE_TYPE_CROSS;
    }
    if ((black > 2U) && (black <= 6U) && ((bits & 0x80U) == 0U)) {
        return LINE_TYPE_LEFT;
    }
    if ((black > 2U) && (black <= 6U) && ((bits & 0x01U) == 0U)) {
        return LINE_TYPE_RIGHT;
    }
    if ((black > 0U) && (black <= 2U)) {
        return LINE_TYPE_STRAIGHT;
    }
    return LINE_TYPE_UNKNOWN;
}
