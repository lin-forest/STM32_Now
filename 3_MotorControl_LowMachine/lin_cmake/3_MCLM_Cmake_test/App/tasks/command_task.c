#include "app_includes.h"

// 外部声明互斥锁
extern osMutexId_t motor_mutexHandle;

void Command_Task(void *argument)
{
    CommandMsg_t cmd; // 直接使用CommandMsg_t接收，因为这个任务不处理uint64_t

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

            // 默认生成成功ACK，后续根据命令类型微调
            ack.type = cmd.type;
            ack.value = cmd.value;
            ack.ok = 1;

            // 处理所有命令，包括CAN
            switch (cmd.type)
            {
                case CMD_FORWARD:
                case CMD_REVERSE:
                case CMD_STOP:
                case CMD_LIST_STATUS:
                case CAN_CMD_STOP:
                    ack.value = 0; // 这些命令的ACK值域为0
                    break;

                case CMD_SET_SPEED:
                case CAN_CMD_SET_SPEED:
                    // ack.value 已被正确设置为命令传入的值
                    break;

                default:
                    ack.ok = 0; // 未知命令，标记为失败
                    ack.value = 0;
                    break;
            }

            /* ================= 2. 投递到 AckQueue ================= */
            // Directly put the address of the AckMsg_t struct into the queue.
            // The queue was created with the correct item size.
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

            /* ================= 3. 分发给电机 ================= */
            if(cmd.type != CMD_NONE)
            {
                // Directly put the address of the CommandMsg_t struct into the queue.
                osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
            }
        }
    }
}