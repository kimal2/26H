#include "My_UartProc.h"

#include "StepMotor_Ctrl.h"
#include "usart.h"

#define CAMERA_UART            huart5
#define CAMERA_FRAME_SIZE      2U

static uint8_t camera_rx_data[CAMERA_FRAME_SIZE];

static HAL_StatusTypeDef Camera_StartReceive(void)
{
    return HAL_UARTEx_ReceiveToIdle_IT(&CAMERA_UART,
                                       camera_rx_data,
                                       CAMERA_FRAME_SIZE);
}

HAL_StatusTypeDef UartReceiveStart(void)
{
    return Camera_StartReceive();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t ball_x;

    if (huart != &CAMERA_UART) {
        return;
    }

    if (size == CAMERA_FRAME_SIZE) {
#if CAMERA_X_BIG_ENDIAN
        ball_x = ((uint16_t)camera_rx_data[0] << 8) |
                 (uint16_t)camera_rx_data[1];
#else
        ball_x = (uint16_t)camera_rx_data[0] |
                 ((uint16_t)camera_rx_data[1] << 8);
#endif
        StepMotor_SetBallX(ball_x);
    }

    (void)Camera_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &CAMERA_UART) {
        (void)Camera_StartReceive();
    }
}
