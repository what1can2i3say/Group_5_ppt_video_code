#include "bsp_uart.h"
#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include "motor.h"
#include <stdint.h>
#include "lidar.h"
#include "usart.h"
#include "bsp_uart.h" // For bluetooth/debug output
#include <string.h>


// Lidar data buffer
#define LIDAR_RX_BUFFER_SIZE 256
uint8_t lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
volatile uint8_t lidar_receiving = 0;


/**
 * @brief Sends a 2-byte command to the Lidar.
 * @param cmd The command to send.
 */
static void lidar_send_cmd(uint16_t cmd)
{
    uint8_t command_bytes[2];
    command_bytes[0] = 0xA5; // A5
    command_bytes[1] = cmd;        // command
    HAL_UART_Transmit(&huart6, command_bytes, 2, 100);
}

/**
 * @brief Initializes the Lidar communication.
 */
void lidar_init(void)
{
    // 确保在CubeMX中已将USART6的DMA模式设置为Circular
    // 启动DMA循环接收
    HAL_UART_Receive_DMA(&huart6, lidar_rx_buffer, LIDAR_RX_BUFFER_SIZE);
}

/**
 * @brief Starts the Lidar scan.
 */
void lidar_start_scan(void)
{
    lidar_send_cmd(RPLIDAR_CMD_SCAN);
    lidar_receiving = 1;
    // 重新启动DMA，以防之前被停止
    HAL_UART_Receive_DMA(&huart6, lidar_rx_buffer, LIDAR_RX_BUFFER_SIZE);
}


/**
 * @brief Stops the Lidar scan.
 */
void lidar_stop_scan(void)
{
    lidar_send_cmd(RPLIDAR_CMD_STOP);
    lidar_receiving = 0;
    // 停止DMA接收
    HAL_UART_DMAStop(&huart6);
}

/**
 * @brief Resets the Lidar.
 */
void lidar_reset(void)
{
    lidar_send_cmd(RPLIDAR_CMD_RESET);
}

/**
 * @brief Gets the health status of the Lidar.
 */
void lidar_get_health(void)
{
    lidar_send_cmd(RPLIDAR_CMD_GET_HEALTH);
    char health_msg[30];
    HAL_UART_Receive_IT(&huart6, (uint8_t*)health_msg, sizeof(health_msg));
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)health_msg, sizeof(health_msg));
}

/**
 * @brief Gets the device information of the Lidar.
 */
void lidar_get_info(void)
{
    lidar_send_cmd(RPLIDAR_CMD_GET_INFO);
    char info_msg[50];
    HAL_UART_Receive_IT(&huart6, (uint8_t*)info_msg, sizeof(info_msg));
    HAL_UART_Transmit_IT(&huart3, (uint8_t*)info_msg, sizeof(info_msg));
}

/**
  * @brief  DMA接收过半回调
  * @param  huart: UART handle.
  * @retval None
  */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6 && lidar_receiving)
    {
        // 处理前半部分数据，使用非阻塞的IT方式发送
        HAL_UART_Transmit_IT(&huart3, lidar_rx_buffer, LIDAR_RX_BUFFER_SIZE / 2);
    }
}

/**
  * @brief  DMA全满回调处理函数
  *         此函数将从 bsp_uart.c 中的主 HAL_UART_RxCpltCallback 调用
  */
void lidar_dma_full_callback_handler(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6 && lidar_receiving)
    {
        // 处理后半部分数据，使用非阻塞的IT方式发送
        HAL_UART_Transmit_IT(&huart3, &lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE / 2], LIDAR_RX_BUFFER_SIZE / 2);
    }
}


