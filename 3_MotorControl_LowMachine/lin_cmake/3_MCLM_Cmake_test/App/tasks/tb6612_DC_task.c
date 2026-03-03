#include "app_includes.h"
#include "pid.h"
#include "can_service.h"
#include "motor_DC_tb6612.h" // Added this include
#include "math.h"   // For fabsf
#include "float.h"  // For FLT_EPSILON

#if (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)

// PID_Controller motor_pid; // 全局，只定义一次
static TB6612_Motor_t tb6612_motor; // Make motor object local to the task
// TB6612_Motor_t tb6612_motor1;

extern PID_Controller motor_pid; // 全局，只定义一次

void TB6612_DC_Task(void *argument)
{
    Motor_PID_Init(); // 初始化参数

    CommandMsg_t cmdMsg;
    // uint64_t queueData; // 用于接收两个队列的数据
    // CanFeedback_t canFeedback;
    // uint8_t feedbackCounter = 0;

    // main.c迁移
    TB6612_Motor_Init(&tb6612_motor, MOTOR1_TIM_HANDLE, MOTOR1_TIM_CHANNEL,\
                MOTOR1_IN1_PORT, MOTOR1_IN1_PIN,\
                MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,\
                MOTOR1_EN_PORT, MOTOR1_EN_PIN, /* EN (如果没有独立的使能引脚，则为 NULL, 0) */\
                MOTOR1_MAX_PWM_OUTPUT, MOTOR1_MAX_SPEED_LOGIC, MOTOR1_MIN_PWM_OUTPUT,\
                MOTOR1_DEAD_ZONE, TB6612_MOTOR_STOP_BRAKE); // Changed Motor_Init and MOTOR_STOP_MODE

    /* Infinite loop */
    for(;;)
    {
        osStatus_t status;
        uint8_t messageProcessed = 0;

        // 1. 从统一的指令队列获取指令 (串口和CAN指令现在都通过MotorQueueHandle发送)
        status = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (status == osOK)
        {
            messageProcessed = 1;

            // 2. 根据指令类型处理
            if (cmdMsg.type == CMD_SET_SPEED || cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
                if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK) {
                    g_motor_status.target_logic_speed = motor_pid.setpoint;
                    osMutexRelease(motor_mutexHandle);
                }
            }
            else if (cmdMsg.type == CMD_STOP || cmdMsg.type == CAN_CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
            }
            // 注意: ACK 消息由各自的命令源任务（如 Command_Task, Can_Service_Task）处理，
            // 此任务只负责执行命令。
        }

        // 3. 执行PID闭环控制 (逻辑不变)
        if (fabsf(motor_pid.setpoint) > FLT_EPSILON) // 优化浮点数比较
        {
            // --- Lock Mutex ---
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                float current_speed = g_motor_status.current_logic_speed; // 使用实际速度作为当前速度
                float output = PID_Compute(&motor_pid, current_speed);
                TB6612_Motor_SetSpeed(&tb6612_motor, (int16_t)output); // Changed Motor_SetSpeed
                g_motor_status.pwm_output = tb6612_motor.pwm_output;

                // --- Release Mutex ---
                osMutexRelease(motor_mutexHandle);
            }
        }
        else
        {
            TB6612_Motor_Stop(&tb6612_motor); // Changed Motor_Stop
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                g_motor_status.pwm_output = 0;
                osMutexRelease(motor_mutexHandle);
            }
        }
        
        // // 4. 周期性发送CAN反馈 (逻辑不变)
        // // if (++feedback_count >= 10)
        // // {
        // //     feedback_count = 0;
        // //     // CanService_SendFeedback(CAN_CMD_SET_SPEED, current_speed); // This was the call
        // // }

        // 如果本次循环没有处理任何消息，就短暂休眠以让出CPU
        if (!messageProcessed) {
            osDelay(5); 
        }
        osDelay(10);
    }
}

#endif // (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)