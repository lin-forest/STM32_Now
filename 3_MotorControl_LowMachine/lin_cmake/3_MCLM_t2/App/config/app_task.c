#include "app_config.h"
#include "app_includes.h"
#include "app_task.h"

// Define the global motor status object
Motor_t g_motors[MOTOR_COUNT] = {
    {
        .hardware = {
            .htim = MOTOR1_PWM_TIM,
            .Channel = MOTOR1_PWM_CH,
            .IN1_Port = MOTOR1_IN1_PORT,
            .IN1_Pin = MOTOR1_IN1_PIN,
            .IN2_Port = MOTOR1_IN2_PORT,
            .IN2_Pin = MOTOR1_IN2_PIN,
            .EN_Port = MOTOR1_STBY_PORT,
            .EN_Pin = MOTOR1_STBY_PIN,
            .MaxPWM = MOTOR1_MAX_PWM_OUTPUT,
            .MaxSpeed = MOTOR1_MAX_SPEED_LOGIC,   // Fix 13: 补充 MaxSpeed，否则 pwmVal 永远为 0
            // .MinPWM = MOTOR1_MIN_PWM_OUTPUT,
            .DeadZone = MOTOR1_DEAD_ZONE,
            // 极性和刹车已经在tb6612.c中绑定
            .Polarity = MOTOR1_POLARITY,
            .StopMode = TB6612_MOTOR_STOP_BRAKE,
        }
    },
    {
        .hardware = {
            .htim = MOTOR2_PWM_TIM,
            .Channel = MOTOR2_PWM_CH,
            .IN1_Port = MOTOR2_IN1_PORT,
            .IN1_Pin = MOTOR2_IN1_PIN,
            .IN2_Port = MOTOR2_IN2_PORT,
            .IN2_Pin = MOTOR2_IN2_PIN,
            .EN_Port = MOTOR2_STBY_PORT,
            .EN_Pin = MOTOR2_STBY_PIN,
            .MaxPWM = MOTOR2_MAX_PWM_OUTPUT,
            .MaxSpeed = MOTOR2_MAX_SPEED_LOGIC,   // Fix 13: 补充 MaxSpeed，否则 pwmVal 永远为 0
            // .MinPWM = MOTOR2_MIN_PWM_OUTPUT,
            .DeadZone = MOTOR2_DEAD_ZONE,
            .Polarity = MOTOR2_POLARITY,
            .StopMode = TB6612_MOTOR_STOP_BRAKE,
        }
    }
};

volatile uint8_t g_logger_enabled = 0;  // 默认关闭，由CAN命令控制
uint8_t tx_buf[128];     // 日志缓存（扩容：logger 最大帧约 62 B，原 64 B 在 enc_abs 极限值时截断 \r\n）[Fix #6]

void Motor_PID_Init(Motor_t *motor)
{
    // 根据电机索引（通过判断硬件地址简单区分）初始化 PID
    float kp = (motor == &g_motors[0]) ? MOTOR1_PID_KP : MOTOR2_PID_KP;
    float ki = (motor == &g_motors[0]) ? MOTOR1_PID_KI : MOTOR2_PID_KI;
    float kd = (motor == &g_motors[0]) ? MOTOR1_PID_KD : MOTOR2_PID_KD;
    float i_limit = (motor == &g_motors[0]) ? MOTOR1_PID_INTEGRAL_LIMIT : MOTOR2_PID_INTEGRAL_LIMIT;
    float o_limit = (motor == &g_motors[0]) ? MOTOR1_PID_OUTPUT_LIMIT : MOTOR2_PID_OUTPUT_LIMIT;
    float ts = (motor == &g_motors[0]) ? MOTOR1_PID_TS : MOTOR2_PID_TS;
    float alpha = (motor == &g_motors[0]) ? MOTOR1_PID_DERIVATIVE_FILTER_ALPHA : MOTOR2_PID_DERIVATIVE_FILTER_ALPHA;

    PID_Init(&(motor->pid), kp, ki, kd, i_limit, o_limit, ts, alpha);
}
