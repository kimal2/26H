#include "graysensor.h"

#define GRAY_SENSOR_SDA_GPIO_PORT  GPIOC
#define GRAY_SENSOR_SDA_PIN        GPIO_PIN_5

#define GRAY_SENSOR_SCL_GPIO_PORT  GPIOC
#define GRAY_SENSOR_SCL_PIN        GPIO_PIN_4
#define GRAY_SENSOR_HALF_PERIOD_US 10U

#define GRAY_SENSOR_WRITE_ADDR \
    ((uint8_t)(GW_GRAY_ADDR_DEFAULT << 1U))

GraySensor_t gray_sensor;

static uint8_t GraySensor_ReadRegister(GraySensor_t *sensor,
                                       uint8_t reg, uint8_t *value)
{
    soft_iic_obj_t *i2c;

    if ((sensor == 0) || (value == 0)) {
        return 1U;
    }
    i2c = &sensor->i2c;

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

    *value = IIC_Read_Byte(i2c, 0U);
    IIC_Stop(i2c);
    return 0U;
}

static uint8_t GraySensor_ReadCurrent(GraySensor_t *sensor, uint8_t *value)
{
    soft_iic_obj_t *i2c;

    if ((sensor == 0) || (value == 0)) {
        return 1U;
    }
    i2c = &sensor->i2c;

    IIC_Start(i2c);
    IIC_Send_Byte(i2c, (uint8_t)(i2c->addr | 0x01U));
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    *value = IIC_Read_Byte(i2c, 0U);
    IIC_Stop(i2c);
    return 0U;
}

static uint8_t GraySensor_SelectCommand(GraySensor_t *sensor, uint8_t command)
{
    soft_iic_obj_t *i2c;

    if (sensor == 0) {
        return 1U;
    }
    i2c = &sensor->i2c;

    IIC_Start(i2c);
    IIC_Send_Byte(i2c, i2c->addr);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Send_Byte(i2c, command);
    if (IIC_Wait_Ack(i2c) != 0U) {
        IIC_Stop(i2c);
        return 1U;
    }

    IIC_Stop(i2c);
    return 0U;
}

uint8_t GraySensor_Init(GraySensor_t *sensor)
{
    uint8_t ping_value = 0U;

    if (sensor == 0) {
        return 1U;
    }

    sensor->digital = 0U;
    sensor->data_valid = 0U;
    sensor->init_ok = 0U;
    soft_iic_init(&sensor->i2c,
                  GRAY_SENSOR_SDA_GPIO_PORT, GRAY_SENSOR_SDA_PIN,
                  GRAY_SENSOR_SCL_GPIO_PORT, GRAY_SENSOR_SCL_PIN,
                  GRAY_SENSOR_WRITE_ADDR);
    soft_iic_set_half_period_us(&sensor->i2c, GRAY_SENSOR_HALF_PERIOD_US);

    if ((GraySensor_ReadRegister(sensor, GW_GRAY_PING, &ping_value) != 0U) ||
        (ping_value != GW_GRAY_PING_OK)) {
        return 1U;
    }
    if (GraySensor_SelectCommand(sensor, GW_GRAY_DIGITAL_MODE) != 0U) {
        return 1U;
    }

    sensor->init_ok = 1U;
    return 0U;
}

uint8_t GraySensor_GetDigital(GraySensor_t *sensor)
{
    uint8_t value = 0U;

    if ((sensor == 0) || (sensor->init_ok == 0U)) {
        return 0U;
    }

    sensor->data_valid = 0U;
    if (GraySensor_ReadCurrent(sensor, &value) != 0U) {
        return sensor->digital;
    }

    sensor->digital = value;
    sensor->data_valid = 1U;
    return sensor->digital;
}
