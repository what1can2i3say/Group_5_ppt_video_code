#include "mpu.h"
#include "run.h"
#include "usart.h"

void MPU6500_WriteReg(uint8_t reg, uint8_t data)
{
	MPU_CS_LOW();
	uint8_t tx[2] = { reg & 0x7F, data }; // 写操作，高位为0
	HAL_SPI_Transmit(&hspi2, tx, 2, 100);
	MPU_CS_HIGH();
}

uint8_t MPU6500_ReadReg(uint8_t reg)
{
	uint8_t tx = reg | 0x80; //读操作，高位为1
	uint8_t rx = 0;
	MPU_CS_LOW();
	HAL_SPI_Transmit(&hspi2, &tx, 1, 100);
	HAL_SPI_Receive(&hspi2, &rx, 1, 100);
	MPU_CS_HIGH();
	return rx;
}

void MPU6500_Init(void)
{
	MPU6500_WriteReg(0x6B, 0x00); // 解除休眠
	HAL_Delay(10);
	MPU6500_WriteReg(0x1A, 0x03); // 配置DLPF
	MPU6500_WriteReg(0x1B, 0x00); // 陀螺仪 ±250dps
	MPU6500_WriteReg(0x1C, 0x00); // 加速度 ±2g

	uint8_t whoami = MPU6500_ReadReg(0x75); // WHO_AM_I
	if (whoami == 0x70)
		printf("MPU6500 SPI2 OK\n");
	else
		printf("MPU6500 SPI2 FAIL, WHO_AM_I=0x%02X\n", whoami);
}

void MPU6500_ReadAccel(int16_t* ax, int16_t* ay, int16_t* az)
{
	uint8_t buf[6];
	int i = 0;
	for (i = 0; i < 6; i++)
		buf[i] = MPU6500_ReadReg(0x3B + i);

	*ax = ((int16_t)buf[0] << 8) | buf[1];
	*ay = ((int16_t)buf[2] << 8) | buf[3];
	*az = ((int16_t)buf[4] << 8) | buf[5];
}

void MPU6500_ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t buf[6];
	int i = 0;
	for (i = 0; i < 6; i++)
		buf[i] = MPU6500_ReadReg(0x43 + i);

	*gx = ((int16_t)buf[0] << 8) | buf[1];
	*gy = ((int16_t)buf[2] << 8) | buf[3];
	*gz = ((int16_t)buf[4] << 8) | buf[5];
}
