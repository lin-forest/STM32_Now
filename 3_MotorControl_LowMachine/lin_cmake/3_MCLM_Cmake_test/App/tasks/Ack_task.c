#include "app_includes.h"

void Ack_Task(void *argument)
{
    AckMsg_t ack;

    for(;;)
    {
        // 阻塞等待 AckQueue
        if(osMessageQueueGet(AckQueueHandle, &ack, NULL, osWaitForever) == osOK)
        {
            char buf[64];

            if(ack.ok)
            {
                switch(ack.type)
                {
                    case CMD_FORWARD:
                        sprintf(buf, "ACK: FORWARD\r\n");
                        break;
                    case CMD_REVERSE:
                        sprintf(buf, "ACK: REVERSE\r\n");
                        break;
                    case CMD_STOP:
                        sprintf(buf, "ACK: STOP\r\n");
                        break;
                    case CMD_SET_SPEED:
                        sprintf(buf, "ACK: SPEED %d\r\n", ack.value);
                        break;
                    case CMD_LIST_STATUS:
                        sprintf(buf, "ACK: LS\r\n");
                        break;
                    default:
                        sprintf(buf, "ACK: UNKNOWN\r\n");
                        break;
                }
            }
            else
            {
                sprintf(buf, "ACK: UNKNOWN\r\n");
            }

            // 发送到 UART2
            UART2_Print(buf);
        }
    }
}
