#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "usart.h"



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
extern int is_buzy;

#endif // !INTERRUPT_H
