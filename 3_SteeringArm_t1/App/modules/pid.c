#include "pid.h"


/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器结构体的指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param integral_limit 积分项限幅
 * @param output_limit 控制器总输出限幅
 * @param Ts 采样周期
 * @param derivative_filter_alpha 微分项滤波系数 (0.0 - 1.0), 0表示无滤波，1表示完全滤波
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit, float Ts, float derivative_filter_alpha)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->Ts = Ts;
    pid->derivative_filter_alpha = derivative_filter_alpha;

    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_value = 0.0f; // 初始化 prev_value
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

    // 2. 计算积分项 (带条件积分 Anti-windup: 输出饱和时停止积分)
    // 评估当前 P+I 输出是否已饱和，方向与误差相同则锁定积分（防止深度 windup）
    float p_term = pid->Kp * error;
    float pre_output = p_term + pid->Ki * pid->integral;
    uint8_t saturated_pos = (pre_output >= pid->output_limit);
    uint8_t saturated_neg = (pre_output <= -pid->output_limit);
    uint8_t anti_windup_hold = (saturated_pos && error > 0.0f) || (saturated_neg && error < 0.0f);

    if (!anti_windup_hold)
    {
        pid->integral += error * pid->Ts;
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        else if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }

    // 3. 计算微分项 (基于误差变化，并进行滤波)
    // 考虑使用 (current_value - pid->prev_value) / pid->Ts 来计算微分项，避免 setpoint 突变时的微分冲击
    // 但为了保持与原代码逻辑一致，这里仍基于误差变化
    derivative = (error - pid->prev_error) / pid->Ts; // 除以 Ts 得到变化率

    // 简单一阶滤波
    pid->last_derivative = (1.0f - pid->derivative_filter_alpha) * pid->last_derivative + pid->derivative_filter_alpha * derivative;
    derivative = pid->last_derivative; // 使用滤波后的微分项

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

    // 6. 更新历史误差和历史测量值
    pid->prev_error = error;
    pid->prev_value = current_value;

    return output;
}

/**
 * @brief 设置PID控制器的目标值
 * @param pid 指向PID控制器结构体的指针
 * @param new_setpoint 新的目标值
 */
void PID_SetSetpoint(PID_Controller *pid, float new_setpoint)
{
    pid->setpoint = new_setpoint;
}

/**
 * @brief 重置PID控制器的内部状态 (积分项和历史误差)
 * @param pid 指向PID控制器结构体的指针
 */
void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_derivative = 0.0f;
    pid->prev_value = 0.0f;
}