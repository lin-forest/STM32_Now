#include "app_includes.h"
#include "pid.h"
#include "can_service.h"
#include "motor_DC_tb6612.h" // Added this include
#include "math.h"   // For fabsf
#include "float.h"  // For FLT_EPSILON

PID_Controller motor_pid; // 全局，只定义一次
static TB6612_Motor_t tb6612_motor1; // Make motor object local to the task
// TB6612_Motor_t tb6612_motor1;

void TB6612_DC_Task(void *argument)
{
    Motor_PID_Init(); // 初始化参数

    CommandMsg_t cmdMsg;
    // uint64_t queueData; // 用于接收两个队列的数据
    // CanFeedback_t canFeedback;
    // uint8_t feedbackCounter = 0;

    // main.c迁移
    TB6612_Motor_Init(&tb6612_motor1, MOTOR1_TIM_HANDLE, MOTOR1_TIM_CHANNEL,\
                MOTOR1_IN1_PORT, MOTOR1_IN1_PIN,\
                MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,\
                MOTOR1_EN_PORT, MOTOR1_EN_PIN, /* EN (如果没有独立的使能引脚，则为 NULL, 0) */\
                MOTOR1_MAX_PWM_OUTPUT, MOTOR1_MAX_SPEED_LOGIC, MOTOR1_MIN_PWM_OUTPUT,\
                MOTOR1_DEAD_ZONE, TB6612_MOTOR_STOP_BRAKE); // Changed Motor_Init and MOTOR1_STOP_MODE

    /* Infinite loop */
    for(;;)
    {
        osStatus_t serialStatus, canStatus;
        uint8_t messageProcessed = 0;
        AckMsg_t ack; // 在循环开始时声明 ack 消息

        // 1. 检查来自串口的指令 (MotorQueue)
        serialStatus = osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0);
        if (serialStatus == osOK)
        {
            messageProcessed = 1;
            // memcpy(&cmdMsg, &queueData, sizeof(CommandMsg_t));
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
            }
            // 串口命令的 ACK 已经在 Command_Task 中处理，这里不需要重复发送
        }

        // 2. 检查来自CAN的指令 (CanMotorCmdQueue)
        canStatus = osMessageQueueGet(CanMotorCmdQueueHandle, &cmdMsg, NULL, 0);
        if (canStatus == osOK)
        {
            messageProcessed = 1;
            // memcpy(&cmdMsg, &queueData, sizeof(CommandMsg_t));
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
            }

            // 为 CAN 命令生成 ACK 消息并发送
            ack.type = cmdMsg.type;
            ack.value = cmdMsg.value;
            ack.ok = 1; // 假设 CAN 命令处理成功

            // 获取电机当前状态，需要互斥锁保护
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                ack.current_logic_speed = g_motor_status.current_logic_speed;
                ack.pwm_output = g_motor_status.pwm_output;
                osMutexRelease(motor_mutexHandle);
            }
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
        }

        // 3. 执行PID闭环控制 (逻辑不变)
        if (fabsf(motor_pid.setpoint) > FLT_EPSILON) // 优化浮点数比较
        {
            // --- Lock Mutex ---
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                float current_speed = g_motor_status.current_logic_speed; // 使用实际速度作为当前速度
                float output = PID_Compute(&motor_pid, current_speed);
                TB6612_Motor_SetSpeed(&tb6612_motor1, (int16_t)output); // Changed Motor_SetSpeed
                g_motor_status.pwm_output = tb6612_motor1.pwm_output;

                // --- Release Mutex ---
                osMutexRelease(motor_mutexHandle);
            }
        }
        else
        {
            TB6612_Motor_Stop(&tb6612_motor1); // Changed Motor_Stop
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                g_motor_status.pwm_output = 0;
                osMutexRelease(motor_mutexHandle);
            }
        }
        
        // // 4. 周期性发送CAN反馈 (逻辑不变)
        // if (++feedback_count >= 10)
        // {
        //     feedback_count = 0;
        //     // CanService_SendFeedback(CAN_CMD_SET_SPEED, current_speed); // This was the call
        // }

        // 如果本次循环没有处理任何消息，就短暂休眠以让出CPU
        if (!messageProcessed) {
            osDelay(5); 
        }
        osDelay(10);
    }
}

void Motor_PID_Init(void)
{
    // float integral_limit = 500.0f;\n    // float output_limit = 100.0f; // 将输出限幅改为100.0f
    // 初始化PID控制器
    PID_Init(&motor_pid, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, MOTOR_PID_INTEGRAL_LIMIT, MOTOR_PID_OUTPUT_LIMIT, PID_TS, PID_DERIVATIVE_FILTER_ALPHA);
}