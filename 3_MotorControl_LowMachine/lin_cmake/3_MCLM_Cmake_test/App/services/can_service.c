// // #include "can.h"
// #include "app_includes.h"
// // #include "stm32f1xx_hal_gpio.h"
// #include "can_service.h" // Add this line to include its own header
// #include "stm32f1xx_hal_can.h" // 确保这个头文件被包含，它定义了 CAN_HandleTypeDef
// // #include "app_config.h" // Include app_config.h for CAN_MOTOR_CMD_STDID

// extern CAN_HandleTypeDef hcan; // 声明 hcan 为外部变量

// // CAN 命令消息队列句柄 (在 can_service.h 中声明为 extern)
// extern osMessageQueueId_t CanMotorCmdQueueHandle;

// /**
//   * @brief CAN 服务初始化
//   * @param None
//   * @retval None
//   */
// void CanService_Init(void)
// {
//     CAN_FilterTypeDef sFilterConfig;

//     // 1. 配置 CAN 过滤器
//     sFilterConfig.FilterBank = CAN_FILTER_BANK;
//     sFilterConfig.FilterMode = CAN_FILTER_MODE;
//     sFilterConfig.FilterScale = CAN_FILTER_SCALE;
//     sFilterConfig.FilterIdHigh = (CAN_MOTOR_CMD_STDID << 5); // 匹配标准ID
//     sFilterConfig.FilterIdLow = 0x0000;
//     sFilterConfig.FilterMaskIdHigh = (0x7FF << 5); // 匹配所有标准ID
//     sFilterConfig.FilterMaskIdLow = 0x0000;
//     sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO;
//     sFilterConfig.FilterActivation = CAN_FILTER_ACTIVATION;
//     sFilterConfig.SlaveStartFilterBank = 14; // 对于双CAN，从CAN1的过滤器组14开始

//     if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     // 2. 启动 CAN
//     if (HAL_CAN_Start(&hcan) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     // 3. 激活 CAN RX 接收中断
//     if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//     {
//         Error_Handler();
//     }

//     // // 4. 创建 CAN 命令消息队列 (如果尚未创建)
//     // if (CanMotorCmdQueueHandle == NULL) {
//     //     CanMotorCmdQueueHandle = osMessageQueueNew(CAN_CMD_QUEUE_SIZE, sizeof(CommandMsg_t), NULL);
//     //     if (CanMotorCmdQueueHandle == NULL) {
//     //         Error_Handler(); // 队列创建失败
//     //     }
//     // }
// }

// /**
//   * @brief CAN 服务发送反馈
//   * @param feedback 反馈结构体指针
//   * @retval None
//   */
// void CanService_SendFeedback(CanFeedback_t *feedback)
// {
//     // TODO: 实现 CAN 反馈发送逻辑
//     // 可以将 feedback 结构体打包成 CAN 帧并通过 HAL_CAN_AddTxMessage 发送
// }

// // CAN 接收中断回调（将 CAN 指令转换为现有 CommandMsg_t 入队）
// void CanService_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rxHeader;
//     uint8_t rxData[8];
//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//     {
//         // 确认ID (虽然过滤器已经保证了)
//         if (rxHeader.StdId == CAN_MOTOR_CMD_STDID) // 使用宏定义的 CAN ID
//         {
//             CommandMsg_t cmdMsg;
//             // 根据原始 can.c 的逻辑，假设所有 0x40 ID 的消息都是设置速度命令
//             cmdMsg.type = CAN_CMD_SET_SPEED;
//             // 速度值从 rxData[1] 获取，并转换为 int16_t
//             cmdMsg.value = (int16_t)((int8_t)rxData[1]);

//             // 使用 CMSIS API 从中断发送队列
//             osMessageQueuePut(CanMotorCmdQueueHandle, &cmdMsg, 0U, 0U);
//         }
//     }
// }

// // CAN 接收中断回调（将 CAN 指令转换为现有 CommandMsg_t 入队）
// // void CanService_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 重命名函数
// // {
// //     CAN_RxHeaderTypeDef rxHeader;
// //     uint8_t rxData[8];
// //     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
// //     {
// //         // 确认ID (虽然过滤器已经保证了)
// //         if (rxHeader.StdId == CAN_MOTOR_CMD_STDID) // 使用宏定义的 CAN ID
// //         {
// //             CommandMsg_t cmdMsg;
// //             // 假设 rxData[0] 是命令类型，rxData[1] 是参数值
// //             cmdMsg.type = (CommandType_t)rxData[0];

// //             switch (cmdMsg.type)
// //             {
// //                 case CAN_CMD_SET_SPEED:
// //                     cmdMsg.value = (int16_t)((int8_t)rxData[1]); // 假设速度值是 int8_t
// //                     break;
// //                 case CAN_CMD_STOP:
// //                     cmdMsg.value = 0; // 停止命令通常没有参数
// //                     break;
// //                 // 可以根据需要添加其他 CAN 命令的处理
// //                 default:
// //                     // 未知 CAN 命令，可以忽略或记录错误
// //                     return;
// //             }

// //             // 使用 CMSIS API 从中断发送队列
// //             osMessageQueuePut(CanMotorCmdQueueHandle, &cmdMsg, 0U, 0U);
// //         }
// //     }
// // }