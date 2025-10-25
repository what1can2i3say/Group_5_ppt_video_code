#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

void motor_init(void);
int16_t EncoderA_Get(void);
int16_t EncoderB_Get(void);
void MotorA_SetSpeed(int16_t speed);
void MotorB_SetSpeed(int16_t speed);
void MotorA_PID_Update(int set_speed);
void MotorB_PID_Update(int set_speed);

void motor_gpio_ctrl(GPIO_PinState a1, GPIO_PinState a2, GPIO_PinState b1, GPIO_PinState b2);

#endif
