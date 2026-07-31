#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#include "soft_i2c.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_RawVector_t;

typedef struct {
    float x;
    float y;
    float z;
} MPU6050_Vector_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} MPU6050_Quaternion_t;

typedef struct {
    soft_iic_obj_t i2c;

    MPU6050_RawVector_t acc_raw;
    MPU6050_RawVector_t gyro_raw;
    int16_t temp_raw;

    MPU6050_Vector_t acc;       /* g */
    MPU6050_Vector_t gyro;      /* degree/s */
    float temperature;          /* degree Celsius */

    MPU6050_Vector_t gyro_bias; /* degree/s */
    MPU6050_Quaternion_t q;
    MPU6050_Vector_t integral;

    float roll;                 /* degree */
    float pitch;                /* degree */
    float yaw;                  /* degree, drifts without a magnetometer */

    float accel_scale;
    float gyro_scale;
    uint8_t data_valid;
    uint8_t init_ok;
} MPU6050_t;

extern MPU6050_t imu;

uint8_t MPU6050_Init(MPU6050_t *dev);
uint8_t MPU6050_WhoAmI(MPU6050_t *dev);
uint8_t MPU6050_ReadAll(MPU6050_t *dev);
uint8_t MPU6050_Update(MPU6050_t *dev, float dt);
void MPU6050_CalibrateGyro(MPU6050_t *dev, uint16_t samples);
void MPU6050_ResetAttitude(MPU6050_t *dev);
void MPU6050_ComputeAttitude(MPU6050_t *dev, float dt);

#endif
