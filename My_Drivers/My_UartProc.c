#include "My_UartProc.h"
#include "usart.h"
#include "Emm_V5.h"
#include "PTU_Ctrl.h"
#include "string.h"
#include "stdbool.h"
#include "ringbuffer.h"

/* S_CPOS : cmd[i] = 0x36; ++i; break;	// 读取电机实时位置 */
#define CMD_CPOS                0x36

#define USART_CAM               huart5

#define RXLEN_PAN_TILT          8
#define RXLEN_CAM               8

#define HDMA_CAM                hdma_uart5_rx
#define HDMA_PAN                hdma_usart2_rx
#define HDMA_TILT               hdma_uart4_rx

#define RINGBUFFER_LEN          128

static uint8_t rxData[RXLEN_CAM];
static uint8_t rxDataPan[RXLEN_PAN_TILT];
static uint8_t rxDataTilt[RXLEN_PAN_TILT];

uint8_t pool_cam[RINGBUFFER_LEN];
uint8_t pool_pan[RINGBUFFER_LEN];
uint8_t pool_tilt[RINGBUFFER_LEN];

struct rt_ringbuffer ring_cam;
struct rt_ringbuffer ring_pan;
struct rt_ringbuffer ring_tilt;


void UartReceiveStart(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&USART_CAM);
    __HAL_UART_ENABLE_IT(&USART_CAM,UART_IT_IDLE);

    __HAL_UART_CLEAR_IDLEFLAG(M_Pan.huart);
    __HAL_UART_ENABLE_IT(M_Pan.huart, UART_IT_IDLE);

    __HAL_UART_CLEAR_IDLEFLAG(M_Tilt.huart);
    __HAL_UART_ENABLE_IT(M_Tilt.huart, UART_IT_IDLE);
    

    rt_ringbuffer_init(&ring_cam, pool_cam, RINGBUFFER_LEN);
    rt_ringbuffer_init(&ring_pan, pool_pan, RINGBUFFER_LEN);
    rt_ringbuffer_init(&ring_tilt, pool_tilt, RINGBUFFER_LEN);

    HAL_UARTEx_ReceiveToIdle_DMA(&USART_CAM, rxData, RXLEN_CAM);
    HAL_UARTEx_ReceiveToIdle_DMA(M_Pan.huart, rxDataPan, RXLEN_PAN_TILT);
    HAL_UARTEx_ReceiveToIdle_DMA(M_Tilt.huart, rxDataTilt, RXLEN_PAN_TILT);

    __HAL_DMA_DISABLE_IT(&HDMA_CAM,DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&HDMA_PAN,DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&HDMA_TILT,DMA_IT_HT);
}

/* 环形队列解析数据 */
void UartBuffTask(uint8_t time)
{
    static uint32_t lasttick=0;
    if(HAL_GetTick()-lasttick<time) return;
    lasttick=HAL_GetTick();

    uint8_t buf[RXLEN_PAN_TILT];

    if(rt_ringbuffer_data_len(&ring_cam) >= 8)
    {
        rt_ringbuffer_get(&ring_cam, buf, 8);
        float pan_deg, tilt_deg;
        memcpy(&pan_deg, buf, 4);
        memcpy(&tilt_deg, buf + 4, 4);
        PTU_SetTarget(pan_deg, tilt_deg);
    }

    if(rt_ringbuffer_data_len(&ring_pan)>=8)
    {
        rt_ringbuffer_get(&ring_pan, buf, 8);
        uint8_t sign = buf[2];
        uint32_t raw = (uint32_t)buf[3] << 24
                     | (uint32_t)buf[4] << 16
                     | (uint32_t)buf[5] << 8
                     | (uint32_t)buf[6];

         int32_t intdeg = sign ? -(int32_t)raw : raw;
        M_Pan.current_angle = (float)intdeg * 360.0f / 65536.0f;
    }

    if(rt_ringbuffer_data_len(&ring_tilt) >= 8) 
    {
        rt_ringbuffer_get(&ring_tilt, buf, 8);
        uint8_t sign = buf[2];
        uint32_t raw = (uint32_t)buf[3] << 24
                     | (uint32_t)buf[4] << 16
                     | (uint32_t)buf[5] << 8
                     | (uint32_t)buf[6];
         int32_t intdeg = sign ? -(int32_t)raw : raw;
        M_Tilt.current_angle = (float)intdeg * 360.0f / 65536.0f;
    }



}


/* 回调接收 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &USART_CAM)
    {
        if(Size==8)
        {
            rt_ringbuffer_put_force(&ring_cam, rxData, Size);
        }
        else
        {
            // 长度不对（可能是上电杂波或丢包），直接清空环形队列，强制重新对齐
            rt_ringbuffer_reset(&ring_cam); 
        }
        
        HAL_UARTEx_ReceiveToIdle_DMA(&USART_CAM, rxData, RXLEN_CAM);
        __HAL_DMA_DISABLE_IT(&HDMA_CAM,DMA_IT_HT);
    }

    if(huart == M_Pan.huart)
    {
        if(rxDataPan[0]==M_Pan.ID&&rxDataPan[1]==CMD_CPOS)
        {
            if(Size==8) rt_ringbuffer_put_force(&ring_pan, rxDataPan, Size);

            //HAL_UART_Transmit_DMA(&huart5, rxDataPan, Size);
        }
        
        HAL_UARTEx_ReceiveToIdle_DMA(M_Pan.huart, rxDataPan, RXLEN_PAN_TILT);
        __HAL_DMA_DISABLE_IT(&HDMA_PAN,DMA_IT_HT);
    }

    if(huart == M_Tilt.huart)
    {
        if(rxDataTilt[0]==M_Tilt.ID&&rxDataTilt[1]==CMD_CPOS)
        {
            if(Size==8) rt_ringbuffer_put_force(&ring_tilt, rxDataTilt, Size);

            //HAL_UART_Transmit_DMA(&huart5, rxDataTilt, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(M_Tilt.huart, rxDataTilt, RXLEN_PAN_TILT);
        __HAL_DMA_DISABLE_IT(&HDMA_TILT,DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &USART_CAM)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&USART_CAM, rxData, RXLEN_CAM);
        __HAL_DMA_DISABLE_IT(&HDMA_CAM, DMA_IT_HT);
    }

    if (huart == M_Pan.huart)
    {

        HAL_UARTEx_ReceiveToIdle_DMA(M_Pan.huart, rxDataPan, RXLEN_PAN_TILT);
        __HAL_DMA_DISABLE_IT(&HDMA_PAN, DMA_IT_HT);
    }

    if (huart == M_Tilt.huart)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(M_Tilt.huart, rxDataTilt, RXLEN_PAN_TILT);
        __HAL_DMA_DISABLE_IT(&HDMA_TILT,DMA_IT_HT);
    }
}
