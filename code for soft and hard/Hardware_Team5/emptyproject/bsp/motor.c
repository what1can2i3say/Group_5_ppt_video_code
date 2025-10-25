#include "motor.h"
#include "tim.h"
#include "pid.h"

PID_t pid_A, pid_B;

int abs(int n)
{
	if(n > 0)
	{
		return n;
	}
	return -n;
}

void motor_init(void)
{
	PID_Init(&pid_A, 0.8, 0.02, 0.1);
  PID_Init(&pid_B, 0.8, 0.02, 0.1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void motor_gpio_ctrl(GPIO_PinState a1, GPIO_PinState a2, GPIO_PinState b1, GPIO_PinState b2)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, a1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, a2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, b1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, b2);
}

int16_t EncoderA_Get(void) {
    int16_t val = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return val;
}

int16_t EncoderB_Get(void) {
    int16_t val = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return val;
}

void MotorA_SetSpeed(int16_t speed) {
    uint32_t pwm = abs(speed) * (htim3.Init.Period + 1) / 100;
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

void MotorB_SetSpeed(int16_t speed) {
    uint32_t pwm = abs(speed) * (htim3.Init.Period + 1) / 100;
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void MotorA_PID_Update(int set_speed) {
		pid_A.SetSpeed = set_speed;
    int speed = EncoderA_Get();
    float pwm = PID_Calc(&pid_A, speed);
    MotorA_SetSpeed((int16_t)(pwm > 100 ? 100 : pwm < -100 ? -100 : pwm));
}

void MotorB_PID_Update(int set_speed) {
		pid_B.SetSpeed = set_speed;
    int speed = EncoderB_Get();
    float pwm = PID_Calc(&pid_B, speed);
    MotorB_SetSpeed((int16_t)(pwm > 100 ? 100 : pwm < -100 ? -100 : pwm));
}

