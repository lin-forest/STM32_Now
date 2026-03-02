// // ===== 新架构实现 =====

// #include "app_includes.h"
// #include "pid.h"
// #include "motor_DC_at8236.h"
// #include "math.h"
// #include "float.h"
// #include "stm32f1xx_hal_tim.h"

// #if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)

// // 静态电机对象
// static At8236_Motor_t at8236_A;

// // 全局PID控制器
// extern PID_Controller motor_pid;

// // AT8236电机任务（新架构）
// void AT8236_DC_Task(void *argument)
// {
//     Motor_PID_Init();

//     CommandMsg_t cmdMsg;

//     // 初始化电机参数
//     At8236_Motor_Init(&at8236_A, 
//                       MOTOR1_TIM_HANDLE, 
//                       MOTOR1_PWM_CHANNEL1,
//                       MOTOR1_PWM_CHANNEL2,
//                       MOTOR1_MAX_PWM_OUTPUT, 
//                       MOTOR1_MAX_SPEED_LOGIC);
//     at8236_A.MinPwm = MOTOR1_MIN_PWM_OUTPUT;

//     for (;;)
//     {
//         // 只从统一队列获取命令
//         if (osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0) == osOK)
//         {
//             // 统一处理命令类型
//             switch (cmdMsg.type)
//             {
//                 case CMD_SET_SPEED:
//                     motor_pid.setpoint = (float)cmdMsg.value;
//                     if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
//                         g_motor_status.target_logic_speed = motor_pid.setpoint;
//                         osMutexRelease(motor_mutexHandle);
//                     }
//                     break;
//                 case CMD_STOP:
//                     motor_pid.setpoint = 0.0f;
//                     if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
//                         g_motor_status.target_logic_speed = 0.0f;
//                         osMutexRelease(motor_mutexHandle);
//                     }
//                     break;
//                 default:
//                     // 其他命令类型可扩展
//                     break;
//             }
//         }

//         // PID控制循环
//         if (fabsf(motor_pid.setpoint) > FLT_EPSILON)
//         {
//             if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
//             {
//                 float current_speed = g_motor_status.current_logic_speed;
//                 float output = PID_Compute(&motor_pid, current_speed);
//                 At8236_Motor_SetSpeed(&at8236_A, (int16_t)output);
//                 g_motor_status.pwm_output = at8236_A.pwm_output;
//                 osMutexRelease(motor_mutexHandle);
//             }
//         }
//         else
//         {
//             At8236_Motor_Stop(&at8236_A, AT8236_STOP_BRAKE);
//             if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
//             {
//                 g_motor_status.pwm_output = 0;
//                 osMutexRelease(motor_mutexHandle);
//             }
//         }

//         osDelay(10); // 固定周期运行
//     }
// }

// #endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)


// 260301

#include "app_includes.h"
#include "pid.h"
#include "motor_DC_at8236.h"
#include "math.h"
#include "float.h"
#include "stm32f1xx_hal_tim.h"

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)

// Make the motor object local and static to this task
static At8236_Motor_t at8236_A;

// PID_Controller is now globally defined, so we don't need a local one.
extern PID_Controller motor_pid;

void AT8236_DC_Task(void *argument)
{
    // Initialize PID controller (assuming Motor_PID_Init is globally available)
    Motor_PID_Init();

    CommandMsg_t cmdMsg;

    // Initialize the AT8236 motor with parameters from app_config.h
    // Note: AT8236 uses two PWM channels. We need to define these in app_config.h
    // For now, I'll use placeholders like MOTOR1_TIM_CHANNEL_2
    At8236_Motor_Init(&at8236_A, 
                      MOTOR1_TIM_HANDLE, 
                      MOTOR1_PWM_CHANNEL1, // Corresponds to IN1
                      MOTOR1_PWM_CHANNEL2,      // Corresponds to IN2 - Placeholder!
                      MOTOR1_MAX_PWM_OUTPUT, 
                      MOTOR1_MAX_SPEED_LOGIC);
    at8236_A.MinPwm = MOTOR1_MIN_PWM_OUTPUT;

    for (;;)
    {
        osStatus_t serialStatus, canStatus;
        uint8_t messageProcessed = 0;
        AckMsg_t ack;

        // 1. Check for commands from the serial queue
        serialStatus = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (serialStatus == osOK)
        {
            messageProcessed = 1;
            if (cmdMsg.type == CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = motor_pid.setpoint;
                    osMutexRelease(motor_mutexHandle);
                }
            }
            else if (cmdMsg.type == CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
                 if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = 0.0f;
                    osMutexRelease(motor_mutexHandle);
                }
            }
        }

        // 2. Check for commands from the CAN queue
        canStatus = osMessageQueueGet(CanMotorCmdQueueHandle, &cmdMsg, NULL, 0);
        if (canStatus == osOK)
        {
            messageProcessed = 1;
            if (cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = motor_pid.setpoint;
                    osMutexRelease(motor_mutexHandle);
                }
            }
            else if (cmdMsg.type == CAN_CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = 0.0f;
                    osMutexRelease(motor_mutexHandle);
                }
            }

            // Generate and send ACK for CAN command
            ack.type = cmdMsg.type;
            ack.value = cmdMsg.value;
            ack.ok = 1;
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                ack.current_logic_speed = g_motor_status.current_logic_speed;
                ack.pwm_output = g_motor_status.pwm_output;
                osMutexRelease(motor_mutexHandle);
            }
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
        }

        // 3. Execute PID control loop
        if (fabsf(motor_pid.setpoint) > FLT_EPSILON)
        {
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                float current_speed = g_motor_status.current_logic_speed;
                float output = PID_Compute(&motor_pid, current_speed);
                At8236_Motor_SetSpeed(&at8236_A, (int16_t)output);
                g_motor_status.pwm_output = at8236_A.pwm_output; // Update global status
                osMutexRelease(motor_mutexHandle);
            }
        }
        else
        {
            At8236_Motor_Stop(&at8236_A, AT8236_STOP_BRAKE);
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                g_motor_status.pwm_output = 0;
                osMutexRelease(motor_mutexHandle);
            }
        }

        if (!messageProcessed) {
            osDelay(5); 
        }
        osDelay(10);
    }
}

#endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)