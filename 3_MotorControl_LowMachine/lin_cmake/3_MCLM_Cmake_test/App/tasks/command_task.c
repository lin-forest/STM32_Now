#include "app_includes.h"

// 外部声明电机对象和互斥锁
extern TB6612_Motor_t tb6612_motor1;
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
                ack.current_logic_speed = tb6612_motor1.current_logic_speed;
                ack.pwm_output = tb6612_motor1.pwm_output;
                osMutexRelease(motor_mutexHandle);
            }

            // 只处理非CAN命令
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
                    ack.type  = CMD_STOP;
                    ack.value = 0;
                    ack.ok    = 1;
                    break;

                case CMD_SET_SPEED:
                    ack.type  = CMD_SET_SPEED;
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