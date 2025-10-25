#include "bsp_uart.h"
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "lidar.h" // 确保雷达头文件已包含
#include "mpu6500.h"
#include <string.h> // 用于 strlen

uint8_t bluetooth_data;
uint8_t sending_speed_info = 0;
uint8_t sending_mcu_info = 0; 


void uart_init()
{
    HAL_UART_Receive_IT(&huart3, &bluetooth_data, 1);
}

/**
 * @brief 安全地通过蓝牙发送非阻塞消息
 * @param message 要发送的字符串
 */
void safe_ble_transmit(const char* message)
{
        // 使用非阻塞的IT方式发送
        HAL_UART_Transmit_IT(&huart3, (uint8_t*)message, strlen(message));
}

/**
 * @brief 处理蓝牙接收到的数据，包括电机和雷达控制
 * @param data 从蓝牙接收到的单个字节命令
 */
void HandleBluetoothData(uint8_t data) {
     switch (data) {
        // --- 电机控制 ---
        case 'F':  // Forward
            MotorA_SetSpeed(60);
            MotorB_SetSpeed(-60);
            safe_ble_transmit("MOTOR: FORWARD\r\n");
            break;
        case 'B':  // Backward
            MotorA_SetSpeed(-60);
            MotorB_SetSpeed(60);
            safe_ble_transmit("MOTOR: BACKWARD\r\n");
            break;
        case 'L':  // Left
            MotorA_SetSpeed(55);
            MotorB_SetSpeed(55);
            safe_ble_transmit("MOTOR: LEFT\r\n");
            break;
        case 'R':  // Right
            MotorA_SetSpeed(-55);
            MotorB_SetSpeed(-55);
            safe_ble_transmit("MOTOR: RIGHT\r\n");
            break;
        case 'S':  // Stop
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            safe_ble_transmit("MOTOR: STOP\r\n");
            break;

        // --- 雷达控制 (新增) ---
        case '1': // 开始雷达扫描
            lidar_start_scan();
            safe_ble_transmit("LIDAR: SCANNING STARTED\r\n");
            break;
        case '2': // 停止雷达扫描
            lidar_stop_scan();
            safe_ble_transmit("LIDAR: SCANNING STOPPED\r\n");
            break;
        case '3': // 获取雷达健康状态
            lidar_get_health();
            safe_ble_transmit("LIDAR: HEALTH CHECK REQUESTED\r\n");
            break;
        case '4': // 获取雷达设备信息
            lidar_get_info();
            safe_ble_transmit("LIDAR: INFO REQUESTED\r\n");
            break;
        case '5': // 复位雷达
            lidar_reset();
            safe_ble_transmit("LIDAR: RESETTING\r\n");
            break;
        case 'I':  // 速度发送的开关
            sending_speed_info = !sending_speed_info; // 切换状态
            if (sending_speed_info) {
                safe_ble_transmit("SPEED INFO: ON\r\n");
            } else {
                safe_ble_transmit("SPEED INFO: OFF\r\n");
            }
            break;
            // --- 发送MCU信息 ---
        case 'M':
            sending_mcu_info = !sending_mcu_info; // 切换状态
            if (sending_mcu_info) {
                safe_ble_transmit("MCU INFO: ON\r\n");
            } else {
                safe_ble_transmit("MCU INFO: OFF\r\n");
            }
            break;
        default:
            safe_ble_transmit("UNKNOWN COMMAND\r\n");
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        HandleBluetoothData(bluetooth_data);
        HAL_UART_Receive_IT(&huart3, &bluetooth_data, 1);
    }
    // 调用雷达的DMA全满回调处理器
    lidar_dma_full_callback_handler(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // // --- 高频PID心跳 (每10ms) ---
    // if (htim->Instance == TIM6)
    // {
    //     Motor_PID_Loop_Update(); // 调用PID主循环
    // }

    // --- 低频数据上报 (每1s, 如果您还需要的话) ---
    if (htim->Instance == TIM7)
    {
        if (sending_mcu_info) {
            MPU6500_Angle angle;
            MPU6500_Compute_Angles(&angle);
            char buffer[200];
            snprintf(buffer, sizeof(buffer), "MCU INFO: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n",
                     angle.roll, angle.pitch, angle.yaw);
            safe_ble_transmit(buffer);  // 发送MCU信息
        }
        if (sending_speed_info) {
            float speed_A = EncoderA_Get();
            float speed_B = EncoderB_Get();
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "SPEED: %f %f\r\n", speed_A, speed_B);
            safe_ble_transmit(buffer);  // 发送速度信息
        }
    }
}