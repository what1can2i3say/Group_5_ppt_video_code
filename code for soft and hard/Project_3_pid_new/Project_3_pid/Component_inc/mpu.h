#ifndef MPU_H
#define MPU_H
#include "stm32f4xx.h"
#include <stdio.h>
#include "spi.h"

#define MPU_CS_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define MPU_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define DEG2RAD(x) ((x) * 3.1415926f / 180.0f)
#define GYRO_SENS 131.0f      // ∂‘”¶ °¿250°„/s
#define DT        0.1f  
extern int is_turning;

void MPU6500_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6500_ReadReg(uint8_t reg);
void MPU6500_Init(void);
void MPU6500_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6500_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);
void GyroTurn_Start(float target_rad,float current_speed);
void GyroTurn_Update(void);








#endif // !MPU_H
