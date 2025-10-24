/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rader.h"
#include "blue.h"
#include "reprint.h"
#include "interrupt.h"
#include <stdlib.h>
#include <string.h>
#include "run.h"
#include "mpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float gz_bias = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
	DATA_TYPE_LIDAR = 1,
	DATA_TYPE_IMU   = 2,
	DATA_TYPE_WHEEL = 3,
	ODOM=4
} DataType;
	
#pragma pack(push,1)
typedef struct {
	uint16_t header; //header
	uint8_t type; //1=LIDAR, 2 =IMU 3-WHEEL
	float x; //lidar angle / leftspeed / x acceleration
	float y; //lidar distance/rightspeed/y acceleration
	float z; //z Angular velocity
}SensorData;
#pragma pack(pop)

#define KP  0.6f
#define KI  0.0f
#define KD  0.6f

#define MAX_TURN_SPEED  60.0f   // 最大电机速度（PWM值）
#define MIN_TURN_SPEED  32.0f
#define ANGLE_TOLERANCE 2.0f
#define INTEGRAL_LIMIT  200.0f
static float last_diff = 0.0f;
static float integral = 0.0f;


void Car_Turning_Control(float yaw, float target_yaw)
{
	float diff = yaw - target_yaw;
	
	// 将角度误差限制到 [-180, 180]
	if (diff > 180.0f) diff -= 360.0f;
	if (diff < -180.0f) diff += 360.0f;

	// 计算 PID
	float derivative = diff-last_diff;
	integral += diff;
	if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
	if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
	last_diff = diff;

	float output = KP * diff + KI * integral + KD * derivative;

	// 限幅
	if (output > MAX_TURN_SPEED) output = MAX_TURN_SPEED;
	if (output < -MAX_TURN_SPEED) output = -MAX_TURN_SPEED;

	// 避免死区
	if (fabs(output) < MIN_TURN_SPEED && fabs(diff) > ANGLE_TOLERANCE)
		output = (output > 0 ? MIN_TURN_SPEED : -MIN_TURN_SPEED);

	Set_LeftMotor_Speed(output); // 左轮正转表示右转
	Set_RightMotor_Speed(-output*0.95); // 右轮反转表示右转

	// 角度到达 -> 停止
	if (fabs(diff) < ANGLE_TOLERANCE)
	{
		Set_LeftMotor_Speed(0);
		Set_RightMotor_Speed(0);

		integral = 0;
		last_diff = 0;

		car_state = STATE_IDLE;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	int16_t ax, ay, az, gx, gy, gz;
	float gx_rad, gy_rad, gz_rad;;
	start_bluetooth_receive();
	RPLIDAR_UART_DMA_Start();
	Rader_Start();
	Motor_PWM_Start();
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	MPU6500_Init();
	
	
	int i = 0;
	for (i = 0; i < 200; i++) {
		MPU6500_ReadGyro(&gx, &gy, &gz);
		gz_bias += gz;
		HAL_Delay(5);
	}
	gz_bias /= 200.0f;
	int left = current_speed;
	int right = current_speed;
	//	HAL_Delay(2000);
	printf("test\n");
	run_state = STOP;
	is_buzy = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{    
		static uint32_t last_tick = 0;
		uint32_t now = HAL_GetTick();
		float dt = (now - last_tick) / 1000.0f;
		last_tick = now;
		Update_Left_Encoder_Speed(dt);
		Update_Right_Encoder_Speed(dt);
		
		MPU6500_ReadGyro(&gx, &gy, &gz);
		gz_rad = (gz - gz_bias) / 131.0f/180*3.14;
		
		UpdateYaw(gz_rad/3.14*180 ,dt);
//		if (car_state == STATE_TURNING)
//		{
//			float diff = yaw - target_yaw;
//			if (diff > 180.0f) diff -= 360.0f;
//			if (diff < -180.0f) diff += 360.0f;
//			if ((turn_dir == 1&&diff>=-0.5f) || (turn_dir == -1&&diff<=0.5f))
//			{
//				Set_LeftMotor_Speed(0);
//				Set_RightMotor_Speed(0);
//				car_state = STATE_IDLE;
//			}
//		}
		
		if (car_state == STATE_TURNING)
		{
			Car_Turning_Control(yaw, target_yaw);
		}
		
		
		
//		if (!is_buzy)
//		{
//			Update_Left_Encoder_Speed(dt);
//			Update_Right_Encoder_Speed(dt);
//			SensorData data;
//			data.header = 0xAA55;
//			data.type = DATA_TYPE_WHEEL;
//			data.x = left_speed;
//			data.y = right_speed;
//			data.z = 0;
//			HAL_UART_Transmit(&huart3, (uint8_t*)&data, sizeof(data), 100);
//		}
//		if (!is_buzy)
//		{
//			MPU6500_ReadAccel(&ax, &ay, &az);
//			MPU6500_ReadGyro(&gx, &gy, &gz);
//			gx_rad = gx / 131.0f;
//			gy_rad = gy / 131.0f;
//			gz_rad = (gz - gz_bias) / 131.0f;
//			SensorData data_2;
//			data_2.header = 0xAA55;
//			data_2.type = DATA_TYPE_IMU;
//			data_2.x = ax;
//			data_2.y = ay;
//			data_2.z = gz_rad;
//			HAL_UART_Transmit(&huart3, (uint8_t*)&data_2, sizeof(data_2), 100);
//		}
		
		if (run_state == RUN_F)
		{
			if (fabs(1.17*left_speed) > fabs(right_speed))
			{
				left -= 1;
				right += 1;
				Set_LeftMotor_Speed(left);
				Set_RightMotor_Speed(right);
			}
			if (fabs(1.17*left_speed) < fabs(right_speed))
			{
				left += 1;
				right -= 1;
				Set_LeftMotor_Speed(left);
				Set_RightMotor_Speed(right);
			}
		}
		if (run_state == RUN_B)
		{
			
			if (fabs(left_speed) > fabs(right_speed))
			{
				left -= 1;
				right += 1;
				Set_LeftMotor_Speed(-left);
				Set_RightMotor_Speed(-right);
			}
			if (fabs(left_speed) < fabs(right_speed))
			{
				left += 1;
				right -= 1;
				Set_LeftMotor_Speed(-left);
				Set_RightMotor_Speed(-right);
			}
		}
		if (run_state == STOP)
		{
			left = right = current_speed;
		}
		testxy(left_speed, right_speed, dt,gz_rad);
		if (!is_buzy)
		{
			SensorData data_4; 
			data_4.header = 0xAA55;
			data_4.type = ODOM;
			data_4.x = est.x;
			data_4.y = est.y;
			data_4.z = est.th;
			HAL_UART_Transmit(&huart3, (uint8_t*)&data_4, sizeof(data_4), 100);
		}
		
		static SensorData data_3[100];
		if (ready&&!is_buzy)
		{
			ready = 0;
			is_buzy = 1;
			int i;
			for (i = 0; i < 25; i++) {
				    data_3[i].header = 0xAA55;
					data_3[i].type = DATA_TYPE_LIDAR;
					data_3[i].x = lidar_points[i*5].angle;
					data_3[i].y = lidar_points[i*5].distance;
					data_3[i].z = 0;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)&data_3, sizeof(data_3));
			
		}
		
		HAL_Delay(10);
		

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
