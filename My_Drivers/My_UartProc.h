#ifndef __MYUART_H
#define __MYUART_H

#include "usart.h"

void UartReceiveStart(void);
void UART_IdleHandler(UART_HandleTypeDef *huart);
void UartBuffTask(uint8_t time);

#endif
