#include "mpu6050.h"

#include <math.h>

#define MPU6050_SDA_GPIO_PORT        GPIOA
#define MPU6050_SDA_PIN              GPIO_PIN_3

#define MPU6050_SCL_GPIO_PORT        GPIOA
#define MPU6050_SCL_PIN              GPIO_PIN_4

#define MPU6050_I2C_WRITE_ADDR       0xD0U
#define MPU6050_WHO_AM_I_VALUE       0x68U

#define MPU6050_REG_SMPLRT_DIV       0x19U
#define MPU6050_REG_CONFIG           0x1AU
#define MPU6050_REG_GYRO_CONFIG      0x1BU
#define MPU6050_REG_ACCEL_CONFIG     0x1CU
#define MPU6050_REG_INT_PIN_CFG      0x37U
#define MPU6050_REG_INT_ENABLE       0x38U
#define MPU6050_REG_ACCEL_XOUT_H     0x3BU
#define MPU6050_REG_USER_CTRL        0x6AU
#define MPU6050_REG_PWR_MGMT_1       0x6BU
#define MPU6050_REG_PWR_MGMT_2       0x6CU
#define MPU6050_REG_FIFO_ENABLE      0x23U
#define MPU6050_REG_WHO_AM_I         0x75U

#define MPU6050_DEG_TO_RAD           0.01745329252f
#define MPU6050_RAD_TO_DEG           57.29577951f
#define MPU6050_GYRO_DEADBAND_DPS    0.15f
#define MPU6050_MAHONY_KP            1.0f
#define MPU6050_MAHONY_KI            0.0f
#define MPU6050_INTEGRAL_LIMIT       0.05f
#define MPU6050_ACCEL_NORM_MIN_SQ    0.7225f
#define MPU6050_ACCEL_NORM_MAX_SQ    1.3225f

MPU6050_t imu;

static float MPU6050_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t MPU6050_WriteReg(MPU6050_t *dev, uint8_t reg, uint8_t value)
{
    soft_iic_obj_t *i2c;

    if (dev == 0) {
        return 1U;
    }
    i2c = &dev->i2c;

    IIC_Start(i2c);
    IIC_Send_Byte(i2c, i2c->addr);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Send_Byte(i2c, reg);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Send_Byte(i2c, value);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Stop(i2c);
    return 0U;
}

static uint8_t MPU6050_ReadBytes(MPU6050_t *dev, uint8_t reg,
                                 uint8_t *data, uint8_t length)
{
    uint8_t i;
    soft_iic_obj_t *i2c;

    if ((dev == 0) || (data == 0) || (length == 0U)) {
        return 1U;
    }
    i2c = &dev->i2c;

    IIC_Start(i2c);
    IIC_Send_Byte(i2c, i2c->addr);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Send_Byte(i2c, reg);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Start(i2c);
    IIC_Send_Byte(i2c, (uint8_t)(i2c->addr | 0x01U));
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    for (i = 0U; i < length; i++) {
        data[i] = IIC_Read_Byte(i2c, (uint8_t)(i + 1U < length));
    }

    IIC_Stop(i2c);
    return 0U;
}

uint8_t MPU6050_WhoAmI(MPU6050_t *dev)
{
    uint8_t value = 0xFFU;

    if (MPU6050_ReadBytes(dev, MPU6050_REG_WHO_AM_I, &value, 1U) != 0U) {
        return 0xFFU;
    }
    return value;
}

uint8_t MPU6050_Init(MPU6050_t *dev)
{
    if (dev == 0) {
        return 1U;
    }

    dev->init_ok = 0U;
    dev->data_valid = 0U;
    soft_iic_init(&dev->i2c,
                  MPU6050_SDA_GPIO_PORT, MPU6050_SDA_PIN,
                  MPU6050_SCL_GPIO_PORT, MPU6050_SCL_PIN,
                  MPU6050_I2C_WRITE_ADDR);

    HAL_Delay(10U);
    if (MPU6050_WhoAmI(dev) != MPU6050_WHO_AM_I_VALUE) {
        return 1U;
    }

    if (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x80U) != 0U) {
        return 1U;
    }
    HAL_Delay(100U);

    if ((MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x01U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_2, 0x00U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x07U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x03U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CONFIG, 0x18U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x00U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_INT_ENABLE, 0x00U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_USER_CTRL, 0x00U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_FIFO_ENABLE, 0x00U) != 0U) ||
        (MPU6050_WriteReg(dev, MPU6050_REG_INT_PIN_CFG, 0x80U) != 0U)) {
        return 1U;
    }

    dev->accel_scale = 2.0f / 32768.0f;
    dev->gyro_scale = 2000.0f / 32768.0f;
    dev->gyro_bias.x = 0.0f;
    dev->gyro_bias.y = 0.0f;
    dev->gyro_bias.z = 0.0f;
    dev->acc.x = 0.0f;
    dev->acc.y = 0.0f;
    dev->acc.z = 1.0f;
    dev->gyro.x = 0.0f;
    dev->gyro.y = 0.0f;
    dev->gyro.z = 0.0f;
    dev->temperature = 0.0f;
    MPU6050_ResetAttitude(dev);
    dev->init_ok = 1U;

    return 0U;
}

uint8_t MPU6050_ReadAll(MPU6050_t *dev)
{
    uint8_t buffer[14];

    if (dev == 0) {
        return 1U;
    }

    dev->data_valid = 0U;
    if (MPU6050_ReadBytes(dev, MPU6050_REG_ACCEL_XOUT_H,
                          buffer, (uint8_t)sizeof(buffer)) != 0U) {
        return 1U;
    }

    dev->acc_raw.x = (int16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
    dev->acc_raw.y = (int16_t)(((uint16_t)buffer[2] << 8U) | buffer[3]);
    dev->acc_raw.z = (int16_t)(((uint16_t)buffer[4] << 8U) | buffer[5]);
    dev->temp_raw = (int16_t)(((uint16_t)buffer[6] << 8U) | buffer[7]);
    dev->gyro_raw.x = (int16_t)(((uint16_t)buffer[8] << 8U) | buffer[9]);
    dev->gyro_raw.y = (int16_t)(((uint16_t)buffer[10] << 8U) | buffer[11]);
    dev->gyro_raw.z = (int16_t)(((uint16_t)buffer[12] << 8U) | buffer[13]);

    dev->acc.x = dev->acc_raw.x * dev->accel_scale;
    dev->acc.y = dev->acc_raw.y * dev->accel_scale;
    dev->acc.z = dev->acc_raw.z * dev->accel_scale;
    dev->gyro.x = dev->gyro_raw.x * dev->gyro_scale;
    dev->gyro.y = dev->gyro_raw.y * dev->gyro_scale;
    dev->gyro.z = dev->gyro_raw.z * dev->gyro_scale;
    dev->temperature = ((float)dev->temp_raw / 340.0f) + 36.53f;
    dev->data_valid = 1U;

    return 0U;
}

void MPU6050_CalibrateGyro(MPU6050_t *dev, uint16_t samples)
{
    uint16_t i;
    uint16_t valid_samples = 0U;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    if ((dev == 0) || (dev->init_ok == 0U) || (samples == 0U)) {
        return;
    }

    dev->gyro_bias.x = 0.0f;
    dev->gyro_bias.y = 0.0f;
    dev->gyro_bias.z = 0.0f;
    for (i = 0U; i < samples; i++) {
        if (MPU6050_ReadAll(dev) == 0U) {
            sum_x += dev->gyro.x;
            sum_y += dev->gyro.y;
            sum_z += dev->gyro.z;
            valid_samples++;
        }
        HAL_Delay(2U);
    }

    if (valid_samples == 0U) {
        return;
    }
    dev->gyro_bias.x = sum_x / valid_samples;
    dev->gyro_bias.y = sum_y / valid_samples;
    dev->gyro_bias.z = sum_z / valid_samples;
    (void)MPU6050_ReadAll(dev);
    MPU6050_ResetAttitude(dev);
}

void MPU6050_ResetAttitude(MPU6050_t *dev)
{
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    float acc_yz;

    if (dev == 0) {
        return;
    }

    acc_yz = sqrtf((dev->acc.y * dev->acc.y) +
                   (dev->acc.z * dev->acc.z));
    roll = atan2f(dev->acc.y, dev->acc.z);
    pitch = atan2f(-dev->acc.x, acc_yz);

    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;
    cos_roll = cosf(half_roll);
    sin_roll = sinf(half_roll);
    cos_pitch = cosf(half_pitch);
    sin_pitch = sinf(half_pitch);

    dev->q.w = cos_roll * cos_pitch;
    dev->q.x = sin_roll * cos_pitch;
    dev->q.y = cos_roll * sin_pitch;
    dev->q.z = -sin_roll * sin_pitch;
    dev->integral.x = 0.0f;
    dev->integral.y = 0.0f;
    dev->integral.z = 0.0f;
    dev->roll = roll * MPU6050_RAD_TO_DEG;
    dev->pitch = pitch * MPU6050_RAD_TO_DEG;
    dev->yaw = 0.0f;
}

void MPU6050_ComputeAttitude(MPU6050_t *dev, float dt)
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float norm_sq;
    float inv_norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float qw;
    float qx;
    float qy;
    float qz;
    float sin_pitch;

    if ((dev == 0) || (dev->init_ok == 0U) ||
        (dev->data_valid == 0U) || (dt <= 0.0f) || (dt > 0.05f)) {
        return;
    }
    dev->data_valid = 0U;

    gx = dev->gyro.x - dev->gyro_bias.x;
    gy = dev->gyro.y - dev->gyro_bias.y;
    gz = dev->gyro.z - dev->gyro_bias.z;
    if (fabsf(gx) < MPU6050_GYRO_DEADBAND_DPS) {
        gx = 0.0f;
    }
    if (fabsf(gy) < MPU6050_GYRO_DEADBAND_DPS) {
        gy = 0.0f;
    }
    if (fabsf(gz) < MPU6050_GYRO_DEADBAND_DPS) {
        gz = 0.0f;
    }
    gx *= MPU6050_DEG_TO_RAD;
    gy *= MPU6050_DEG_TO_RAD;
    gz *= MPU6050_DEG_TO_RAD;

    ax = dev->acc.x;
    ay = dev->acc.y;
    az = dev->acc.z;
    norm_sq = (ax * ax) + (ay * ay) + (az * az);

    if ((norm_sq >= MPU6050_ACCEL_NORM_MIN_SQ) &&
        (norm_sq <= MPU6050_ACCEL_NORM_MAX_SQ)) {
        inv_norm = 1.0f / sqrtf(norm_sq);
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;

        vx = 2.0f * ((dev->q.x * dev->q.z) -
                     (dev->q.w * dev->q.y));
        vy = 2.0f * ((dev->q.w * dev->q.x) +
                     (dev->q.y * dev->q.z));
        vz = (dev->q.w * dev->q.w) - (dev->q.x * dev->q.x) -
             (dev->q.y * dev->q.y) + (dev->q.z * dev->q.z);

        ex = (ay * vz) - (az * vy);
        ey = (az * vx) - (ax * vz);
        ez = (ax * vy) - (ay * vx);

        dev->integral.x = MPU6050_Clamp(
            dev->integral.x + (MPU6050_MAHONY_KI * ex * dt),
            -MPU6050_INTEGRAL_LIMIT, MPU6050_INTEGRAL_LIMIT);
        dev->integral.y = MPU6050_Clamp(
            dev->integral.y + (MPU6050_MAHONY_KI * ey * dt),
            -MPU6050_INTEGRAL_LIMIT, MPU6050_INTEGRAL_LIMIT);
        dev->integral.z = MPU6050_Clamp(
            dev->integral.z + (MPU6050_MAHONY_KI * ez * dt),
            -MPU6050_INTEGRAL_LIMIT, MPU6050_INTEGRAL_LIMIT);

        gx += (MPU6050_MAHONY_KP * ex) + dev->integral.x;
        gy += (MPU6050_MAHONY_KP * ey) + dev->integral.y;
        gz += (MPU6050_MAHONY_KP * ez) + dev->integral.z;
    }

    qw = dev->q.w;
    qx = dev->q.x;
    qy = dev->q.y;
    qz = dev->q.z;
    dev->q.w += 0.5f * dt * ((-qx * gx) - (qy * gy) - (qz * gz));
    dev->q.x += 0.5f * dt * (( qw * gx) + (qy * gz) - (qz * gy));
    dev->q.y += 0.5f * dt * (( qw * gy) - (qx * gz) + (qz * gx));
    dev->q.z += 0.5f * dt * (( qw * gz) + (qx * gy) - (qy * gx));

    norm_sq = (dev->q.w * dev->q.w) + (dev->q.x * dev->q.x) +
              (dev->q.y * dev->q.y) + (dev->q.z * dev->q.z);
    if (norm_sq <= 0.0f) {
        MPU6050_ResetAttitude(dev);
        return;
    }
    inv_norm = 1.0f / sqrtf(norm_sq);
    dev->q.w *= inv_norm;
    dev->q.x *= inv_norm;
    dev->q.y *= inv_norm;
    dev->q.z *= inv_norm;

    dev->roll = atan2f(
        2.0f * ((dev->q.w * dev->q.x) + (dev->q.y * dev->q.z)),
        1.0f - (2.0f * ((dev->q.x * dev->q.x) +
                        (dev->q.y * dev->q.y)))) * MPU6050_RAD_TO_DEG;

    sin_pitch = 2.0f * ((dev->q.w * dev->q.y) -
                        (dev->q.z * dev->q.x));
    sin_pitch = MPU6050_Clamp(sin_pitch, -1.0f, 1.0f);
    dev->pitch = asinf(sin_pitch) * MPU6050_RAD_TO_DEG;

    dev->yaw = atan2f(
        2.0f * ((dev->q.w * dev->q.z) + (dev->q.x * dev->q.y)),
        1.0f - (2.0f * ((dev->q.y * dev->q.y) +
                        (dev->q.z * dev->q.z)))) * MPU6050_RAD_TO_DEG;
}

uint8_t MPU6050_Update(MPU6050_t *dev, float dt)
{
    if ((dev == 0) || (dev->init_ok == 0U) ||
        (dt <= 0.0f) || (dt > 0.05f)) {
        return 1U;
    }
    if (MPU6050_ReadAll(dev) != 0U) {
        return 1U;
    }
    MPU6050_ComputeAttitude(dev, dt);
    return 0U;
}
