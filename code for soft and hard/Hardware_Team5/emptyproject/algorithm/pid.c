#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->SetSpeed = 0; pid->CurSpeed = 0;
    pid->Err = 0; pid->LastErr = 0;
    pid->Integral = 0; pid->Output = 0;
}

float PID_Calc(PID_t *pid, float current_speed) {
    pid->CurSpeed = current_speed;
    pid->Err = pid->SetSpeed - pid->CurSpeed;
    pid->Integral += pid->Err;
    float derivative = pid->Err - pid->LastErr;
    pid->Output = pid->Kp * pid->Err + pid->Ki * pid->Integral + pid->Kd * derivative;
    pid->LastErr = pid->Err;
    return pid->Output;
}
