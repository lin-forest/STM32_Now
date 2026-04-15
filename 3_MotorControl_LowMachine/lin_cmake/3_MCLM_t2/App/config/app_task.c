#include "app_includes.h"
#include "app_task.h"

// Define the global motor status object
Motor_t g_motors[MOTOR_COUNT];
volatile uint8_t g_logger_enabled = 0;  // 默认关闭，由CAN命令控制

uint8_t tx_buf[64];      // 日志缓存

void Motor_PID_Init(Motor_t *motor)
{
    // 初始化PID控制器
    PID_Init(&(motor->pid), MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD, 
             MOTOR1_PID_INTEGRAL_LIMIT, MOTOR1_PID_OUTPUT_LIMIT, 
             MOTOR1_PID_TS, MOTOR1_PID_DERIVATIVE_FILTER_ALPHA);
}
