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
    CanService_Init(); // 初始化 CAN 服务（确保在电机初始化前）
    /* USER CODE BEGIN Start_MotorControl */

    CommandMsg_t cmdMsg;
    CanFeedback_t canFeedback;
    uint8_t feedbackCounter = 0; // 反馈周期计数器（每10次循环发送一次）

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
        // 4. 不再永远等待，而是以超时0的方式检查新指令
        // 恢复 osMessageQueueGet 逻辑，确保 cmdMsg 被使用
        if (osMessageQueueGet(MotorQueueHandle, &cmdMsg, NULL, 0) == osOK)
        {
            // 如果收到了新指令，就更新PID的目标值
            if (cmdMsg.type == CMD_SET_SPEED)
            {
                motor_pid.setpoint = (float)cmdMsg.value;
            }
            else if (cmdMsg.type == CMD_STOP)
            {
                motor_pid.setpoint = 0.0f; // 设置目标值为0
            }
        }

        // 新增：处理 CAN 指令队列
        if (xQueueReceive(CanMotorCmdQueueHandle, &cmdMsg, 0) == pdPASS)
        {
            if (cmdMsg.type == CAN_CMD_SET_SPEED) // 使用统一的 CommandType_t
            {
                motor_pid.setpoint = (float)cmdMsg.value;
            }
            else if (cmdMsg.type == CAN_CMD_STOP) // 使用统一的 CommandType_t
            {
                motor_pid.setpoint = 0.0f;
            }
        }

        // 5. 执行PID闭环控制计算（不变）
        if (motor_pid.setpoint != 0.0f)
        {
            float current_speed = motor1.target_logic_speed;
            float output = PID_Compute(&motor_pid, current_speed);
            Motor_SetSpeed(&motor1, (int16_t)output);
        }
        else // 目标值为0时，明确停止电机
        {
            Motor_Stop(&motor1);
        }
        
        // 新增：周期性发送 CAN 反馈（每100ms发送一次，对应10次osDelay(10)）
        if (++feedbackCounter >= 10)
        {
            canFeedback.current_logic_speed = motor1.current_logic_speed;
            canFeedback.pwm_output = motor1.pwm_output;
            canFeedback.motor_state = (motor_pid.setpoint != 0.0f) ? 1 : 0;
            CanService_SendFeedback(&canFeedback);
            feedbackCounter = 0;
        }

        // 6. 以固定周期运行
        osDelay(10); // 保持与编码器测速周期一致
    }
    /* USER CODE END Start_MotorControl */
}

// #include "cmsis_os.h"
// #include "tb6612_DC.h"
// #include "command.h"
// #include "app_task.h"