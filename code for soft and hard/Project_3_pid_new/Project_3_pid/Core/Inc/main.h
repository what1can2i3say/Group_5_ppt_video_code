/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUTPUT_Pin GPIO_PIN_2
#define OUTPUT_GPIO_Port GPIOA
#define INPUT_Pin GPIO_PIN_3
#define INPUT_GPIO_Port GPIOA
#define LEFT_1_Pin GPIO_PIN_6
#define LEFT_1_GPIO_Port GPIOA
#define LEFT_2_Pin GPIO_PIN_7
#define LEFT_2_GPIO_Port GPIOA
#define Blue_R_Pin GPIO_PIN_5
#define Blue_R_GPIO_Port GPIOC
#define RIGHT_1_Pin GPIO_PIN_0
#define RIGHT_1_GPIO_Port GPIOB
#define RIGHT_2_Pin GPIO_PIN_1
#define RIGHT_2_GPIO_Port GPIOB
#define Blue_T_Pin GPIO_PIN_10
#define Blue_T_GPIO_Port GPIOB
#define MPU6500_CS_Pin GPIO_PIN_8
#define MPU6500_CS_GPIO_Port GPIOA
#define Rader_T_Pin GPIO_PIN_9
#define Rader_T_GPIO_Port GPIOA
#define Rader_R_Pin GPIO_PIN_10
#define Rader_R_GPIO_Port GPIOA
#define Encoder_Pin GPIO_PIN_6
#define Encoder_GPIO_Port GPIOB
#define EncoderB7_Pin GPIO_PIN_7
#define EncoderB7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
