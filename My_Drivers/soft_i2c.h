#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx_hal.h"

typedef struct soft_iic_obj_t
{
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
    GPIO_TypeDef *scl_port;
    uint16_t      scl_pin;

    uint8_t addr; /* 8-bit device address, including the R/W bit position. */
    uint32_t half_period_us; /* 0 selects the fast legacy delay loop. */

}soft_iic_obj_t;

static inline void iic_sda_1(soft_iic_obj_t *dev)
{
    HAL_GPIO_WritePin(dev->sda_port, dev->sda_pin, GPIO_PIN_SET);
}

static inline void iic_sda_0(soft_iic_obj_t *dev)
{
    HAL_GPIO_WritePin(dev->sda_port, dev->sda_pin, GPIO_PIN_RESET);
}

static inline void iic_scl_1(soft_iic_obj_t *dev)
{
    HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_SET);
}

static inline void iic_scl_0(soft_iic_obj_t *dev)
{
    HAL_GPIO_WritePin(dev->scl_port, dev->scl_pin, GPIO_PIN_RESET);
}

void soft_iic_init(soft_iic_obj_t *soft_iic_obj, GPIO_TypeDef *sda_port,
                   uint16_t sda_pin, GPIO_TypeDef *scl_port,
                   uint16_t scl_pin, uint8_t addr);
void soft_iic_set_half_period_us(soft_iic_obj_t *soft_iic_obj,
                                 uint32_t half_period_us);

void IIC_Start(soft_iic_obj_t *soft_iic_obj);
void IIC_Stop(soft_iic_obj_t *soft_iic_obj);
void IIC_Send_Byte(soft_iic_obj_t *soft_iic_obj, uint8_t _ucByte);
uint8_t IIC_Read_Byte(soft_iic_obj_t *soft_iic_obj, uint8_t ack);
uint8_t IIC_Wait_Ack(soft_iic_obj_t *soft_iic_obj);
void IIC_Ack(soft_iic_obj_t *soft_iic_obj);
void IIC_NAck(soft_iic_obj_t *soft_iic_obj);

#endif
