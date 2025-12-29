// #include "FreeRTOS.h"
// #include "queue.h"
// #include <stdint.h>
// #include "command.h" // 新增：包含 command.h 以使用 CommandMsg_t
// #include "can_service.h" // 新增：包含 can_service.h 以使用 CanService_Init

// // 队列句柄声明（确保在 freertos.c 中定义）
// extern QueueHandle_t CanMotorCmdQueueHandle; // 将 CanMotorCmdQueue 重命名为 CanMotorCmdQueueHandle 以与 can_service.h 保持一致


// void CanRecvTask(void *argument)
// {
//     CommandMsg_t cmdMsg; // 修改：接收 CommandMsg_t 类型
//     for (;;)
//     {
//         if (xQueueReceive(CanMotorCmdQueueHandle, &cmdMsg, portMAX_DELAY) == pdPASS) // 修改：从 CanMotorCmdQueueHandle 接收
//         {
//             // 这里可以处理接收到的CAN消息
//             // 例如，根据 cmdMsg.type 和 cmdMsg.value 执行相应的电机控制操作
//             // switch (cmdMsg.type)
//             // {
//             // case CMD_SET_SPEED:
//             //     // 处理设置速度命令
//             //     break;
//             // case CMD_FORWARD:
//             //     // 处理前进命令
//             //     break;
//             // // ... 其他命令
//             // default:
//             //     break;
//             // }
//         }
//     }
// }