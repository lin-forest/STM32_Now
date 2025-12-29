#include "pid.h"


/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器结构体的指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param integral_limit 积分项限幅
 * @param output_limit 控制器总输出限幅
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;

    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_derivative = 0.0f; // 初始化 last_derivative
}

/**
 * @brief 计算PID控制器的输出
 * @param pid 指向PID控制器结构体的指针
 * @param current_value 当前测量值
 * @return 控制器的输出量
 */
float PID_Compute(PID_Controller *pid, float current_value)
{
    float error, derivative, output;

    // 1. 计算误差
    error = pid->setpoint - current_value;

    // 2. 计算积分项 (带抗积分饱和)
    float Ts = 0.01f; // 采样周期，10ms
    pid->integral += error * Ts;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }

    // 3. 计算微分项
    derivative = error - pid->prev_error;
    pid->last_derivative = 0.7f * pid->last_derivative + 0.3f * derivative; // 简单一阶滤波
    derivative = pid->last_derivative; // 使用结构体中的 last_derivative

    // 4. 计算总输出
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 5. 输出限幅
    if (output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    // 6. 更新历史误差
    pid->prev_error = error;

    return output;
}