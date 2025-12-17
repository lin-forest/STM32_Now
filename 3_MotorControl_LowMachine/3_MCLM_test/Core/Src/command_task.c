
#include "app_includes.h"

void Command_Task(void *argument)
{
    CommandMsg_t cmd;

    for (;;)
    {
        if (osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever) == osOK)
        {
            /* ================= 1. 生成 ACK 消息 ================= */
            AckMsg_t ack;

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
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

            /* ================= 3. 分发给电机 ================= */
            if(cmd.type != CMD_NONE)
            {
                osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
            }
        }
    }
}


// // 该部分为串口UART2-ISR成功的代码，留存；后改为ack-queue＆ack_task

// #include "app_includes.h"

// void Command_Task(void *argument)
// {
//     CommandMsg_t cmd;

//     for (;;)
//     {
//         if (osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever) == osOK)
//         {
//             /* ========== 1. 发 ACK（命令被系统接纳） ========== */
//             switch (cmd.type)
//             {
//                 case CMD_FORWARD:
//                     UART2_Print("ACK: FORWARD\r\n");
//                     break;
//                 case CMD_REVERSE:
//                     UART2_Print("ACK: REVERSE\r\n");
//                     break;
//                 case CMD_STOP:
//                     UART2_Print("ACK: STOP\r\n");
//                     break;
//                 case CMD_SET_SPEED:
//                 {
//                     char buf[32];
//                     sprintf(buf, "ACK: SPEED %d\r\n", cmd.value);
//                     UART2_Print(buf);
//                     break;
//                 }
//                 case CMD_LIST_STATUS:
//                     UART2_Print("ACK: LS\r\n");
//                     break;
//                 default:
//                     UART2_Print("ACK: UNKNOWN\r\n");
//                     continue;   // 不往下发
//             }

//             /* ========== 2. 分发给电机 ========== */
//             osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
//         }
//     }
// }
