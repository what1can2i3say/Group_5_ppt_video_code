#include "stm32f4xx.h"
#include "usart.h"
#include <stdio.h>

#ifndef RADER_H
#define RADER_H

#define RPLIDAR_DMA_BUF_LEN 512

extern uint8_t rplidar_rx_buf[RPLIDAR_DMA_BUF_LEN];
extern uint16_t rplidar_rx_len;
extern int flag;
typedef struct {
	float angle;
	float distance;
} RPLidarPoint;

extern RPLidarPoint lidar_points[500];
extern int ready;

void RPLIDAR_UART_DMA_Start(void);//启动DMA

void RPLIDAR_ProcessFrame(uint8_t *buf, uint16_t len);//解报文
void Rader_Start(void);//雷达启动
void Rader_Stop(void);//雷达停止








#endif // !RADER_H
