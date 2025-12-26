#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

// 队列句柄声明（确保在 freertos.c 中定义）
extern QueueHandle_t CanMotorCmdQueue;


void CanRecvTask(void *argument)
{
    uint64_t msg;
    for (;;)
    {
        if (xQueueReceive(CanMotorCmdQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            // 这里可以处理接收到的CAN消息
        }
    }
}