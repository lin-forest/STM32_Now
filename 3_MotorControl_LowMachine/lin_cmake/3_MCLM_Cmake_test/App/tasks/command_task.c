

// 260301改动

#include "app_includes.h"
#include "app_task.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"

// 外部声明互斥锁
extern osMutexId_t motor_mutexHandle;

void Command_Task(void *argument)
{
    CommandMsg_t cmd; // 直接使用CommandMsg_t接收，因为这个任务不处理uint64_t

    // Lin_test
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

    for (;;)
    {
        // 只从CommandQueue获取指令
        if (osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever) == osOK)
        {
            /* ================= 1. 生成 ACK 消息 ================= */
            AckMsg_t ack;

            // 默认值
            ack.current_logic_speed = 0;
            ack.pwm_output = 0;

            // 获取电机当前状态，需要互斥锁保护
            if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
            {
                ack.current_logic_speed = g_motor_status.current_logic_speed;
                ack.pwm_output = g_motor_status.pwm_output;
                osMutexRelease(motor_mutexHandle);
            }

            // 现在处理所有来源的命令，包括CAN
            switch (cmd.type)
            {
                case CMD_FORWARD:
                    ack.type  = CMD_FORWARD;
                    ack.value = 0;
                    ack.ok    = 1;
                    break;

                case CMD_REVERSE:
                    ack.type  = CMD_REVERSE;
                    ack.value = 0;
                    ack.ok    = 1;
                    break;

                case CMD_STOP:
                case CAN_CMD_STOP: // 添加CAN_CMD_STOP
                    ack.type  = cmd.type; // 使用原始命令类型
                    ack.value = 0;
                    ack.ok    = 1;
                    break;

                case CMD_SET_SPEED:
                case CAN_CMD_SET_SPEED: // 添加CAN_CMD_SET_SPEED
                    ack.type  = cmd.type; // 使用原始命令类型
                    ack.value = cmd.value;
                    ack.ok    = 1;
                    break;

                case CMD_LIST_STATUS:
                    ack.type  = CMD_LIST_STATUS;
                    ack.value = 0;
                    ack.ok    = 1;
                    break;

                default:
                    ack.type  = CMD_NONE;
                    ack.value = 0;
                    ack.ok    = 0;
                    break;
            }

            // Lin_test
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

            /* ================= 2. 投递到 AckQueue & 3. 分发给电机 ================= */
            // 只要是有效命令，就生成ACK
            if (ack.ok)
            {
                osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

                // 关键：只有电机需要执行的命令，才转发到MotorQueueHandle
                if (cmd.type == CMD_SET_SPEED || cmd.type == CAN_CMD_SET_SPEED ||
                    cmd.type == CMD_STOP      || cmd.type == CAN_CMD_STOP      ||
                    cmd.type == CMD_FORWARD   || cmd.type == CMD_REVERSE)
                {
                    osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
                }
            }
        }
    }
}