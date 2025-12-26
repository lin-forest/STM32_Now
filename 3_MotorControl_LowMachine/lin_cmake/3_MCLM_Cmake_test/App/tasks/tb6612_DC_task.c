#include "app_includes.h"
#include "pid.h"
#include "can_service.h"

PID_Controller motor_pid; // 全局，只定义一次

void Motor_PID_Init(void)
{
    float integral_limit = 500.0f;
    float output_limit = 100.0f; // 将输出限幅改为100.0f
    PID_Init(&motor_pid, 0.4584f, 17.66f, 0.002976f, integral_limit, output_limit);
}

void tb6612_DC_Task(void *argument)
{
    Motor_PID_Init(); // 初始化参数

    CommandMsg_t cmdMsg;
    uint64_t queueData; // 用于接收两个队列的数据
    // CanFeedback_t canFeedback;
    // uint8_t feedbackCounter = 0;

    // main.c迁移
    Motor_Init(&motor1, &htim3, TIM_CHANNEL_1,\
                GPIOB, GPIO_PIN_0,\
                GPIOB, GPIO_PIN_1,\
                GPIOA, GPIO_PIN_7, /* EN (如果没有独立的使能引脚，则为 NULL, 0) */\
                100, 100, 10,
                0, MOTOR_STOP_BRAKE);

    /* Infinite loop */
    for(;;)
    {
        osStatus_t serialStatus, canStatus;
        uint8_t messageProcessed = 0;

        // 1. 检查来自串口的指令 (MotorQueue)
        serialStatus = osMessageQueueGet(MotorQueueHandle, &queueData, NULL, 0);
        if (serialStatus == osOK)
        {
            messageProcessed = 1;
            memcpy(&cmdMsg, &queueData, sizeof(CommandMsg_t));
            if (cmdMsg.type == CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
            }
            else if (cmdMsg.type == CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
            }
        }

        // 2. 检查来自CAN的指令 (CanMotorCmdQueue)
        canStatus = osMessageQueueGet(CanMotorCmdQueueHandle, &queueData, NULL, 0);
        if (canStatus == osOK)
        {
            messageProcessed = 1;
            memcpy(&cmdMsg, &queueData, sizeof(CommandMsg_t));
            if (cmdMsg.type == CAN_CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
            }
            else if (cmdMsg.type == CAN_CMD_STOP)
            {
                motor_pid.setpoint = 0.0f;
            }
        }

        // 3. 执行PID闭环控制 (逻辑不变)
        if (motor_pid.setpoint != 0.0f)
        {
            float current_speed = motor1.target_logic_speed;
            float output = PID_Compute(&motor_pid, current_speed);
            Motor_SetSpeed(&motor1, (int16_t)output);
        }
        else
        {
            Motor_Stop(&motor1);
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

// #include "cmsis_os.h"
// #include "tb6612_DC.h"
// #include "command.h"
// #include "app_task.h"