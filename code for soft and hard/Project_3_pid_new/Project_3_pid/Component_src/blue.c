#include "blue.h"

uint8_t blue_buff[10];
void start_bluetooth_receive(void)
{
	HAL_UART_Receive_IT(&huart3, blue_buff, 3); // 启动接收1字节
}
