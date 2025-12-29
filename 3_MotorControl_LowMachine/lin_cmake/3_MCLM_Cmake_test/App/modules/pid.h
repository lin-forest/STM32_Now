#ifndef __PID_H
#define __PID_H

#include <stdint.h>

// PID 控制器结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float setpoint;         // 目标值
    float integral;         // 积分项累计值
    float prev_error;       // 上一次的误差

    float integral_limit;   // 积分限幅
    float output_limit;     // 输出限幅

    float last_derivative;  // 并在 PID_Compute 中使用 pid->last_derivative

} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);
float PID_Compute(PID_Controller *pid, float current_value);

#endif // __PID_H