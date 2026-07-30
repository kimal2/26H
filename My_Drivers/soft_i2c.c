#include "soft_i2c.h"
#include "ti_msp_dl_config.h"


void soft_iic_init(soft_iic_obj_t *soft_iic_obj, GPIO_Regs *sda_port,
                   uint32_t sda_pin, uint32_t sda_iomux,
                   GPIO_Regs *scl_port, uint32_t scl_pin, uint8_t addr)
{
    soft_iic_obj->sda_port = sda_port;
    soft_iic_obj->sda_pin = sda_pin;
    soft_iic_obj->scl_port = scl_port;
    soft_iic_obj->scl_pin = scl_pin;

    soft_iic_obj->addr = addr;

    /* SysConfig output mode leaves DIN disabled; SDA must also be readable. */
    IOMUX->SECCFG.PINCM[sda_iomux] |= IOMUX_PINCM_INENA_ENABLE;
}

void IIC_Start(soft_iic_obj_t *soft_iic_obj)
{
    /* 当SCL高电平时，SDA出现一个下跳沿表示IIC总线启动信号 */
    iic_sda_1(soft_iic_obj);
    iic_scl_1(soft_iic_obj);
    iic_delay();
    iic_sda_0(soft_iic_obj);
    iic_scl_0(soft_iic_obj);
}

void IIC_Stop(soft_iic_obj_t *soft_iic_obj)
{
    /* 当SCL高电平时，SDA出现一个上跳沿表示IIC总线停止信号 */
    iic_sda_0(soft_iic_obj);
    iic_scl_1(soft_iic_obj);
    iic_delay();
    iic_sda_1(soft_iic_obj);
}

void IIC_Send_Byte(soft_iic_obj_t *soft_iic_obj, uint8_t _ucByte)
{
    uint8_t i;

    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80)
        {
            iic_sda_1(soft_iic_obj);
        }
        else
        {
            iic_sda_0(soft_iic_obj);
        }
        iic_delay();
        iic_scl_1(soft_iic_obj);
        iic_delay();
        iic_scl_0(soft_iic_obj);
        if (i == 7)
        {
            iic_sda_1(soft_iic_obj); // 释放总线
        }
        _ucByte <<= 1;	/* 左移一个bit */
    }

}

uint8_t IIC_Read_Byte(soft_iic_obj_t *soft_iic_obj, uint8_t ack)
{
    uint8_t i;
    uint8_t value = 0;

    /* 读到第1个bit为数据的bit7 */
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        iic_scl_1(soft_iic_obj);
        iic_delay();
        if (DL_GPIO_readPins(soft_iic_obj->sda_port,
                             soft_iic_obj->sda_pin) != 0U)
        {
            value |= 0x01U;
        }
        iic_scl_0(soft_iic_obj);
        iic_delay();
    }

    if (ack == 0U)
    {
        IIC_NAck(soft_iic_obj);
    }
    else
    {
        IIC_Ack(soft_iic_obj);
    }
    return value;
}

uint8_t IIC_Wait_Ack(soft_iic_obj_t *soft_iic_obj)
{
    uint8_t re;

    iic_sda_1(soft_iic_obj);  /* CPU释放SDA总线 */
    iic_delay();
    iic_scl_1(soft_iic_obj);  /* CPU驱动SCL = 1，此时器件会返回ACK应答 */
    iic_delay();
    if (DL_GPIO_readPins(soft_iic_obj->sda_port,
                         soft_iic_obj->sda_pin) != 0U)
    {
        re = 1;
    }
    else
    {
        re = 0;
    }
    iic_scl_0(soft_iic_obj);
    iic_delay();
    return re;
}

void IIC_Ack(soft_iic_obj_t *soft_iic_obj)
{
    iic_sda_0(soft_iic_obj);  /* CPU驱动SDA = 0，发送ACK */
    iic_delay();
    iic_scl_1(soft_iic_obj);  /* 产生第9个时钟 */
    iic_delay();
    iic_scl_0(soft_iic_obj);
    iic_delay();
    iic_sda_1(soft_iic_obj);  /* 释放SDA总线 */
}

void IIC_NAck(soft_iic_obj_t *soft_iic_obj)
{
    iic_sda_1(soft_iic_obj);  /* CPU释放SDA，发送NACK */
    iic_delay();
    iic_scl_1(soft_iic_obj);  /* 产生第9个时钟 */
    iic_delay();
    iic_scl_0(soft_iic_obj);
    iic_delay();
}
