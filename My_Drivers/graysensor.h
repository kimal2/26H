#ifndef GRAYSENSOR_H
#define GRAYSENSOR_H

#include <stdint.h>

#include "gw_grayscale_sensor.h"

typedef struct {
    uint8_t digital;
    uint8_t data_valid;
    uint8_t init_ok;
} GraySensor_t;

extern GraySensor_t gray_sensor;

uint8_t GraySensor_Init(GraySensor_t *sensor);
uint8_t GraySensor_GetDigital(GraySensor_t *sensor);

#endif
