#ifndef MY_UART_PROC_H
#define MY_UART_PROC_H

#include "stm32f4xx_hal.h"

/* Camera sends one uint16_t X coordinate in little-endian binary format. */
#define CAMERA_X_BIG_ENDIAN    0U

HAL_StatusTypeDef UartReceiveStart(void);

#endif
