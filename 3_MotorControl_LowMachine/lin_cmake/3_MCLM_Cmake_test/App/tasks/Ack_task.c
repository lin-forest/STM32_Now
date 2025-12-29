#include "app_includes.h"

void Ack_Task(void *argument)
{
    AckMsg_t ack;

    for(;;)
    {
        // Directly get the AckMsg_t struct from the queue.
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
                        // This case should ideally not be reached if ack.ok is true
                        sprintf(buf, "ACK: UNKNOWN\r\n");
                        break;
                }
            }
            else
            {
                sprintf(buf, "NACK: UNKNOWN COMMAND\r\n"); // Use NACK for clarity
            }

            // Send to UART2
            UART2_Print(buf);
        }
    }
}