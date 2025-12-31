#include "app_includes.h"
#include "app_task.h"

// Define the global motor status object
MotorStatus_t g_motor_status;

// Define the global PID controller object
PID_Controller motor_pid;

// TB6612_Motor_t tb6612_motor1; // REMOVED - This will be local to the task that uses it.
uint8_t tx_buf[64];      // 日志缓存


void Motor_PID_Init(void)
{
    // float integral_limit = 500.0f;\\n    // float output_limit = 100.0f; // 将输出限幅改为100.0f
    // 初始化PID控制器
    PID_Init(&motor_pid, MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD, MOTOR1_PID_INTEGRAL_LIMIT, MOTOR1_PID_OUTPUT_LIMIT, MOTOR1_PID_TS, MOTOR1_PID_DERIVATIVE_FILTER_ALPHA);
}
