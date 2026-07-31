#include "graysensor.h"
#include "stm32f4xx_hal.h"

#define GRAY_SENSOR_WRITE_ADDR       ((uint8_t)(GW_GRAY_ADDR_DEFAULT << 1U))
#define GRAY_IIC_SDA_PIN             GPIO_PIN_5
#define GRAY_IIC_SCL_PIN             GPIO_PIN_4
#define GRAY_IIC_SDA_MODE_SHIFT      10U
#define GRAY_IIC_SCL_MODE_SHIFT      8U
#define GRAY_IIC_DELAY_US            10U

GraySensor_t gray_sensor;

static uint32_t gray_iic_delay_cycles;

static void GrayIIC_Delay(void)
{
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < gray_iic_delay_cycles) {
    }
}

static void GrayIIC_SetSdaInput(void)
{
    GPIOC->MODER &= ~(0x03U << GRAY_IIC_SDA_MODE_SHIFT);
}

static void GrayIIC_SetSdaOutput(void)
{
    GPIOC->MODER = (GPIOC->MODER & ~(0x03U << GRAY_IIC_SDA_MODE_SHIFT)) |
                   (0x01U << GRAY_IIC_SDA_MODE_SHIFT);
}

static void GrayIIC_SdaHigh(void)
{
    GPIOC->BSRR = GRAY_IIC_SDA_PIN;
}

static void GrayIIC_SdaLow(void)
{
    GPIOC->BSRR = (uint32_t)GRAY_IIC_SDA_PIN << 16U;
}

static void GrayIIC_SclHigh(void)
{
    GPIOC->BSRR = GRAY_IIC_SCL_PIN;
}

static void GrayIIC_SclLow(void)
{
    GPIOC->BSRR = (uint32_t)GRAY_IIC_SCL_PIN << 16U;
}

static uint8_t GrayIIC_ReadSda(void)
{
    return ((GPIOC->IDR & GRAY_IIC_SDA_PIN) != 0U) ? 1U : 0U;
}

static void GrayIIC_Start(void)
{
    GrayIIC_SetSdaOutput();
    GrayIIC_SdaHigh();
    GrayIIC_SclHigh();
    GrayIIC_Delay();
    GrayIIC_SdaLow();
    GrayIIC_Delay();
    GrayIIC_SclLow();
    GrayIIC_Delay();
}

static void GrayIIC_Stop(void)
{
    GrayIIC_SetSdaOutput();
    GrayIIC_SdaLow();
    GrayIIC_Delay();
    GrayIIC_SclHigh();
    GrayIIC_Delay();
    GrayIIC_SdaHigh();
    GrayIIC_Delay();
}

static uint8_t GrayIIC_WaitAck(void)
{
    uint8_t nack;

    GrayIIC_SetSdaOutput();
    GrayIIC_SdaHigh();
    GrayIIC_SclHigh();
    GrayIIC_Delay();
    nack = GrayIIC_ReadSda();
    GrayIIC_SclLow();
    GrayIIC_Delay();
    return nack;
}

static uint8_t GrayIIC_SendByte(uint8_t value)
{
    uint8_t bit;

    GrayIIC_SetSdaOutput();
    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            GrayIIC_SdaHigh();
        } else {
            GrayIIC_SdaLow();
        }
        GrayIIC_Delay();
        GrayIIC_SclHigh();
        GrayIIC_Delay();
        GrayIIC_SclLow();
        GrayIIC_Delay();
        value <<= 1U;
    }
    return GrayIIC_WaitAck();
}

static uint8_t GrayIIC_ReceiveByte(void)
{
    uint8_t bit;
    uint8_t value = 0U;

    GrayIIC_SdaHigh();
    GrayIIC_SetSdaInput();
    for (bit = 0U; bit < 8U; bit++) {
        value <<= 1U;
        GrayIIC_SclHigh();
        GrayIIC_Delay();
        if (GrayIIC_ReadSda() != 0U) {
            value |= 0x01U;
        }
        GrayIIC_SclLow();
        GrayIIC_Delay();
    }
    GrayIIC_SetSdaOutput();
    GrayIIC_SdaHigh();
    return value;
}

static void GrayIIC_SendNack(void)
{
    GrayIIC_SetSdaOutput();
    GrayIIC_SdaHigh();
    GrayIIC_Delay();
    GrayIIC_SclHigh();
    GrayIIC_Delay();
    GrayIIC_SclLow();
    GrayIIC_Delay();
}

static void GrayIIC_Init(void)
{
    uint32_t pin_mode_mask;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    pin_mode_mask = (0x03U << GRAY_IIC_SDA_MODE_SHIFT) |
                    (0x03U << GRAY_IIC_SCL_MODE_SHIFT);
    GPIOC->MODER = (GPIOC->MODER & ~pin_mode_mask) |
                   (0x01U << GRAY_IIC_SDA_MODE_SHIFT) |
                   (0x01U << GRAY_IIC_SCL_MODE_SHIFT);
    GPIOC->OTYPER |= GRAY_IIC_SDA_PIN | GRAY_IIC_SCL_PIN;
    GPIOC->PUPDR = (GPIOC->PUPDR & ~pin_mode_mask) |
                   (0x01U << GRAY_IIC_SDA_MODE_SHIFT) |
                   (0x01U << GRAY_IIC_SCL_MODE_SHIFT);
    GPIOC->OSPEEDR |= pin_mode_mask;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    gray_iic_delay_cycles = (SystemCoreClock / 1000000U) *
                            GRAY_IIC_DELAY_US;
    if (gray_iic_delay_cycles == 0U) {
        gray_iic_delay_cycles = 1U;
    }

    GrayIIC_SdaHigh();
    GrayIIC_SclHigh();
}

static uint8_t GrayIIC_ReadRegister(uint8_t register_address, uint8_t *value)
{
    if (value == 0) {
        return 1U;
    }

    GrayIIC_Start();
    if (GrayIIC_SendByte((uint8_t)(GRAY_SENSOR_WRITE_ADDR & 0xFEU)) != 0U) {
        GrayIIC_Stop();
        return 1U;
    }
    if (GrayIIC_SendByte(register_address) != 0U) {
        GrayIIC_Stop();
        return 1U;
    }

    GrayIIC_Start();
    if (GrayIIC_SendByte((uint8_t)(GRAY_SENSOR_WRITE_ADDR | 0x01U)) != 0U) {
        GrayIIC_Stop();
        return 1U;
    }

    *value = GrayIIC_ReceiveByte();
    GrayIIC_SendNack();
    GrayIIC_Stop();
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
    GrayIIC_Init();

    if ((GrayIIC_ReadRegister(GW_GRAY_PING, &ping_value) != 0U) ||
        (ping_value != GW_GRAY_PING_OK)) {
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
    if (GrayIIC_ReadRegister(GW_GRAY_DIGITAL_MODE, &value) != 0U) {
        return sensor->digital;
    }

    sensor->digital = value;
    sensor->data_valid = 1U;
    return sensor->digital;
}
