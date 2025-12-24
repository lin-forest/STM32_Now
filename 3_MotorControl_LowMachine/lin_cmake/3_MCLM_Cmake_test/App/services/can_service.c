// #include <string.h>
// #include "command.h"
// #include "app_task.h"
// #include "stm32f1xx_hal_can.h"

// /* ===================== CAN RX ISR ===================== */
// /* 只做：取帧 → 丢进 CAN RX Queue */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rxHeader;
//     uint8_t data[8];

//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
//         return;

//     /* 投递给 CAN RX Queue（ISR 安全） */
//     osMessageQueuePut(CanRxQueueHandle, data, 0, 0);
// }
