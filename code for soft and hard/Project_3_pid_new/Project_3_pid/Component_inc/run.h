#ifndef RUN_H
#define RUN_H

#include "stm32f4xx.h"
#include <stdio.h>
#include "tim.h"

#define ENCODER_PPR (390*4)
#define CONTROL_PERIOD_MS 20
#define SPEED_STEP 100.0f 

typedef enum
{
	RUN_F, 
	RUN_B, 
	STOP
}RUNState;
extern RUNState run_state;

typedef enum {
	STATE_IDLE,
	// 空闲
	STATE_TURNING    // 正在转弯
} CarState;

#define L_WHEEL 0.18f          // 轮间距 (m)，示例 0.30m
#define DT      0.01f          // 采样周期 s (例如 10ms)
#define Q_POS   1e-4f          // 过程噪声：位置相关（可调）
#define Q_THETA 1e-5f          // 过程噪声：角度相关（可调）
#define R_THETA 5e-3f    // 观测噪声：IMU积分后角度不确定性（可调）
#define M_PI 3.14

/* ---------- 状态与协方差 ---------- */
/* 状态 x = [x, y, theta]^T */
typedef struct {
	float x; // x (m)
	float y; // y (m)
	float th; // theta (rad)
} State;

/* 3x3 矩阵 */
typedef struct {
	float m[3][3];
} Mat3;

/* 3x1 向量 */
typedef struct {
	float v[3];
} Vec3;


extern float left_speed;
extern float right_speed;
extern int left;
extern int right;
extern float current_speed; // 默认占空比（1000 = 100%）

extern float yaw ;
extern float start_yaw ;
extern float target_yaw; 
extern int turn_dir;
extern CarState car_state;
extern State est;
extern int target_angle;
//extern float integral_L;extern float prev_error_L ;
//
//// 右电机 PID 状态
//extern float integral_R ;extern float prev_error_R;
//void Handle_Command(char cmd);
//
//void Motor_PWM_Start(void);
//void Set_LeftMotor_Speed(float speed);
////void Set_RightMotor_Speed(float speed);
//
void Update_Left_Encoder_Speed(float dt);
void Update_Right_Encoder_Speed(float dt);
//
//float PID_Update_L(float target, float actual);
////float PID_Update_R(float target, float actual);
//
//void Set_LeftMotor_Speed(float pwm);
//
////void Set_RightMotor_Speed(float pwm);
//void Gradually_Approach_Target();
////double Read_ADC_Value(void);
void Set_LeftMotor_Speed(int8_t speed);
void Motor_PWM_Start(void);
void Set_RightMotor_Speed(int8_t speed);
void Handle_Command(char* cmd);
double Read_ADC_Value(void);
void UpdateYaw(float gz_dps, float dt);
void StartTurn(float angle, int dir);
void testxy(float v1, float v2, float dt,float gz);
#endif // !RUN_H
