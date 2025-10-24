#include "run.h"
#include "adc.h"
#include "mpu.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
float target_speed; // 目标转速 (单位 rpm)
float current_pwm; // 当前PWM输出占空比(0~100)
float measured_speed;

void Motor_PWM_Start(void) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 左 IN1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 左 IN2
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 右 IN1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // 右 IN2
}

void Set_LeftMotor_Speed(int8_t speed) {
	if (speed > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed * 10); // 正转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else if (speed < 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed * 10); // 反转
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

void Set_RightMotor_Speed(int8_t speed) {
	if (speed > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed * 10); // 正转
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
	else if (speed < 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed * 10); // 反转
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	}
}

int left;
int right;
int target_angle;

float current_speed = 40; // 默认占空比（1000 = 100%）
void Handle_Command(char* cmd) {
	char direction = cmd[0];
	const char* angle = cmd + 1;
	target_angle = atoi(angle);
	if (direction == 'L'&&target_angle != 0)
	{
		printf("L90\r\n");
		StartTurn(target_angle, 1);
	}
	if (direction == 'R'&&target_angle != 0)
	{
		StartTurn(target_angle, -1);
	}
//	if (direction == 'L'&&target_angle == 0)
//	{
//		Set_LeftMotor_Speed(-current_speed);
//		Set_RightMotor_Speed(current_speed);
//		
//	}
//	if (direction == 'R'&&target_angle == 0)
//	{
//		Set_LeftMotor_Speed(current_speed);
//		Set_RightMotor_Speed(-current_speed);
//	}
	if (direction == 'F')
	{
		printf("F\r\n");
		Set_LeftMotor_Speed(+ current_speed);
		Set_RightMotor_Speed(+ (current_speed + 4));
		run_state = RUN_F;
	}
	if (direction == 'S')
	{
		printf("S\r\n");
		Set_LeftMotor_Speed(0);
		Set_RightMotor_Speed(0);
		run_state = STOP;
	}
	if (direction == 'B')
	{
		printf("B\r\n");
		Set_LeftMotor_Speed(-current_speed);
		Set_RightMotor_Speed(-(current_speed + 2));
		run_state = RUN_B;
	}
}


//switch (cmd) {
//case 'F':  // Forward
//	current_speed = 40;
//	Set_LeftMotor_Speed(+ current_speed);
//	Set_RightMotor_Speed(+ (current_speed + 4));
//	break;
//case 'B':  // Backward
//	current_speed = 40;
//	Set_LeftMotor_Speed(-current_speed);
//	Set_RightMotor_Speed(-(current_speed + 2));
//	break;
//case 'L':  // Turn Left
//	current_speed = 40;
//	Set_LeftMotor_Speed(-current_speed);
//	Set_RightMotor_Speed(current_speed);
//	break;
//case 'R':  // Turn Right
//	current_speed = 40;
//	Set_LeftMotor_Speed(current_speed);
//	Set_RightMotor_Speed(-current_speed);
//	break;
//case 'S':  // Stop
//	Set_LeftMotor_Speed(0);
//	Set_RightMotor_Speed(0);
//	break;
//case '1': case '2': case '3': case '4':
//case '5': case '6': case '7': case '8': case '9':
//	current_speed = (cmd - '0') * 10; // 占空比 = 100 ~ 900
//	break;
//default:
//	break;
//}




uint32_t adc_raw = 0;
float adc_voltage = 0;

double Read_ADC_Value(void) {
	HAL_ADC_Start(&hadc1); // 启动 ADC
	HAL_ADC_PollForConversion(&hadc1, 10); // 等待转换完成（阻塞方式）
	adc_raw = HAL_ADC_GetValue(&hadc1); // 读取原始值
	HAL_ADC_Stop(&hadc1); // 停止 ADC

	adc_voltage = (adc_raw / 4095.0f) * 3.3f; // 假设是 12 位 ADC 和 3.3V 参考电压
	return adc_voltage; // 返回电压值
}



int16_t last_left_count = 0;
int16_t last_right_count = 0;

float left_speed = 0;
float right_speed = 0;

void Update_Left_Encoder_Speed(float dt) {
	int16_t current = __HAL_TIM_GET_COUNTER(&htim2); // 左轮编码器接 TIM2
	int16_t delta = current - last_left_count;

	// 处理溢出
	if (delta > 32767) delta -= 65536;
	else if (delta < -32768) delta += 65536;

	last_left_count = current;

	left_speed = -(delta * (1000.0f /(dt*1000))) / ENCODER_PPR*3.14*2*0.034;
	
}

void Update_Right_Encoder_Speed(float dt) {
	int16_t current = __HAL_TIM_GET_COUNTER(&htim4); // 右轮编码器接 TIM4
	int16_t delta = current - last_right_count;

	// 处理溢出
	if (delta > 32767) delta -= 65536;
	else if (delta < -32768) delta += 65536;

	last_right_count = current;

	right_speed = -(delta * (1000.0f / (dt*1000))) / ENCODER_PPR*3.14*2*0.034;

}


extern float gz_bias;
float yaw = 0;
float start_yaw= 0;
float target_yaw = 0;
int turn_dir;
CarState car_state;
RUNState run_state;

void UpdateYaw(float gz_dps,float dt) {
	yaw += gz_dps * dt*10;
	if (yaw > 180.0f) yaw -= 360.0f;
	if (yaw < -180.0f) yaw += 360.0f;
}

void StartTurn(float angle, int dir) {
	start_yaw = yaw;
	target_yaw = start_yaw + dir * angle;
	turn_dir = dir;
	car_state = STATE_TURNING;
	
//	if (dir > 0)
//	{
//		Set_LeftMotor_Speed(-current_speed-1);
//		Set_RightMotor_Speed(current_speed);
//	}
//	if (dir < 0)
//	{
//		Set_LeftMotor_Speed(current_speed+2);
//		Set_RightMotor_Speed(-current_speed);
//	}
}


State est; // 当前估计
Mat3 P; // 当前协方差
float theta_imu_integrated = 0.0f; // IMU 积分得到的角度（用于观测）



void testxy(float v1, float v2, float dt,float gz)
{
	float v = 0.5f * (1.17*v1 + v2);
	float omega = (v2 - v1*1.17) / L_WHEEL;
	float th = est.th;
	est.x += v * cosf(th) * dt;
	est.y += v * sinf(th) * dt;
	est.th += gz*dt*0.8+omega*dt*0.2;

	// normalize angle
	while (est.th > 3.14) est.th -= 6.28;
	while (est.th < -3.14) est.th += 6.28;
	
}
