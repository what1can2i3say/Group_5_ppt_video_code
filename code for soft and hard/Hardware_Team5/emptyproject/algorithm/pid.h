#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float Kp, Ki, Kd;
    float SetSpeed, CurSpeed, Err, LastErr, Integral, Output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Calc(PID_t *pid, float current_speed);

#endif
