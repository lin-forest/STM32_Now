
#include "app_includes.h"

void Command_Task(void *argument)
{
    CommandMsg_t cmd;

    for (;;)
    {
        if (osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever) == osOK)
        {
            /* ========== 1. 发 ACK（命令被系统接纳） ========== */
            switch (cmd.type)
            {
                case CMD_FORWARD:
                    UART2_Print("ACK: FORWARD\r\n");
                    break;
                case CMD_REVERSE:
                    UART2_Print("ACK: REVERSE\r\n");
                    break;
                case CMD_STOP:
                    UART2_Print("ACK: STOP\r\n");
                    break;
                case CMD_SET_SPEED:
                {
                    char buf[32];
                    sprintf(buf, "ACK: SPEED %d\r\n", cmd.value);
                    UART2_Print(buf);
                    break;
                }
                case CMD_LIST_STATUS:
                    UART2_Print("ACK: LS\r\n");
                    break;
                default:
                    UART2_Print("ACK: UNKNOWN\r\n");
                    continue;   // 不往下发
            }

            /* ========== 2. 分发给电机 ========== */
            osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
        }
    }
}


// #include "app_includes.h"

// void Command_Task(void *argument)
// {
//       osDelay(1000);

// }
