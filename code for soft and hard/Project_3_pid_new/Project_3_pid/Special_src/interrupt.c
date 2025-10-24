#include "interrupt.h"
#include "rader.h"
#include "run.h"
extern uint8_t blue_buff[10];
int is_buzy=0;
int is_buzy_wheel = 0;
int is_buzy_imu;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		HAL_UART_Transmit(&huart3, blue_buff, 3, 100); // 回显
		Handle_Command(blue_buff);
		HAL_UART_Receive_IT(&huart3, blue_buff, 3); // 继续接收
	}
	if (huart->Instance == USART1) {
		
		RPLIDAR_ProcessFrame(&rplidar_rx_buf[RPLIDAR_DMA_BUF_LEN / 2], RPLIDAR_DMA_BUF_LEN / 2);
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		RPLIDAR_ProcessFrame(&rplidar_rx_buf[0], RPLIDAR_DMA_BUF_LEN / 2);
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3) {
		is_buzy = 0;
		printf("one time\r\n");
	}
}
