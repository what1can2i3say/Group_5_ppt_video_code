#ifndef __LIDAR_H
#define __LIDAR_H

#include "main.h"

// RPLidar 命令
#define RPLIDAR_CMD_STOP 0x25
#define RPLIDAR_CMD_SCAN 0x20
#define EXPRESS_SCAN 0x82
#define RPLIDAR_CMD_FORCE_SCAN 0x21
#define RPLIDAR_CMD_RESET 0x40
#define RPLIDAR_CMD_GET_INFO 0x50
#define RPLIDAR_CMD_GET_HEALTH 0x52

// 函数声明
void lidar_init(void);
void lidar_start_scan(void);
void lidar_stop_scan(void);
void lidar_reset(void);
void lidar_get_health(void);
void lidar_get_info(void);
void lidar_dma_full_callback_handler(UART_HandleTypeDef *huart);

#endif // __LIDAR_H
