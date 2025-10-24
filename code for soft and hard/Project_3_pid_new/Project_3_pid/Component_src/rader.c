#include "rader.h"
#include "string.h"


uint8_t rplidar_rx_buf[RPLIDAR_DMA_BUF_LEN];
uint16_t rplidar_rx_len=0;

RPLidarPoint lidar_points[500];

int count = 0;
int flag = 0;
int ready=0;
void RPLIDAR_UART_DMA_Start(void)
{
	HAL_UART_Receive_DMA(&huart1, rplidar_rx_buf, RPLIDAR_DMA_BUF_LEN);
	
}

void RPLIDAR_ProcessFrame(uint8_t *buf, uint16_t len)
{
	if (len < 5) 
		return;
//	int i = 0;
//	for ( i = 0; i < len; i++) {
//		printf("%02X ", buf[i]);
//		if ((i + 1) % 16 == 0) printf("\r\n");
//	}
//	if (len % 16 != 0) printf("\r\n");

	int j = 0;
	while (j + 4 < len)
	{
		uint8_t byte0 = buf[j];
		uint8_t byte1 = buf[j + 1];

		uint8_t S = byte0 & 0x01;
		uint8_t NotS = (byte0 >> 1) & 0x01;
		uint8_t C = byte1 & 0x01;

		if ((S ^ NotS) != 1 || C != 1)
		{
			j++;
			continue;
		}

		uint8_t byte2 = buf[j + 2];
		uint8_t byte3 = buf[j + 3];
		uint8_t byte4 = buf[j + 4];

		uint16_t angle_q6 = ((uint16_t)byte2 << 7) | (byte1 >> 1);
		float angle = angle_q6 / 64.0f;

		uint16_t dist_q2 = ((uint16_t)byte4 << 8) | byte3;
		float distance = dist_q2 / 4.0f;

		lidar_points[count].angle = angle;
		lidar_points[count].distance = distance;

		
		

		count++;
		if (count >= 500)
		{
			count = 0;
			ready = 1;
		}
		j += 5;
	}
}

void Rader_Start()
{
	uint8_t cmd_scan[2] = { 0xA5, 0x20 };
	HAL_UART_Transmit(&huart1, cmd_scan, 2, 100);
	HAL_Delay(10);
}

void Reader_Stop()
{
	uint8_t cmd_stop[2] = { 0xA5, 0x25 };
	HAL_UART_Transmit(&huart1, cmd_stop, 2, 100);
	HAL_Delay(10);
}