// #include "can.h"
// #include "app_includes.h"
// #include "stm32f1xx_hal_gpio.h"
// // #include "can_service.h" // Add this line to include its own header

// // QueueHandle_t CanMotorCmdQueueHandle;

// void CanService_Init(void)
// {
//     MX_CAN_Init(); // Changed from MX_CAN1_Init
// //     CanMotorCmdQueueHandle = xQueueCreate(CAN_CMD_QUEUE_SIZE, sizeof(CommandMsg_t));
    
//     // 配置 CAN 滤波（仅接收电机相关消息 ID，示例 ID=0x100）
//     CAN_FilterTypeDef sFilterConfig;

//     sFilterConfig.FilterBank = 0;
//     sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//     sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//     sFilterConfig.FilterIdHigh = 0x0000;
//     sFilterConfig.FilterIdLow = 0x0000;
//     sFilterConfig.FilterMaskIdHigh = 0x0000;
//     sFilterConfig.FilterMaskIdLow = 0x0000;
//     sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//     sFilterConfig.FilterActivation = ENABLE; // Changed from FilterActivate
//     sFilterConfig.SlaveStartFilterBank = 0;

//     if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) // Changed from &hcan1
//     {
//         Error_Handler();
//     }

//     if (HAL_CAN_Start(&hcan) != HAL_OK) // Changed from &hcan1
//     {
//         Error_Handler();
//     }

//     if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // Changed from &hcan1
//     {
//         Error_Handler();
//     }
// }

// // CAN 接收中断回调（将 CAN 指令转换为现有 CommandMsg_t 入队）
// // 删除整个 HAL_CAN_RxFifo0MsgPendingCallback 函数定义
// // {
// //     CAN_RxHeaderTypeDef RxHeader;
// //     uint8_t RxData[8];

// //     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
// //     {
// //         // 这行代码（如果存在）就是最直观的物理指示
// //         // 每次有 CAN 消息（任何ID）被硬件接收并触发中断，PC14 的 LED 就会闪烁一次
// //         HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); 

// //         // 这里检查消息的 ID 是否为我们关心的 0x07B
// //         if (RxHeader.StdId == 0x07B)
// //         {
// //             CommandMsg_t msg;
// //             msg.type = (CommandType_t)RxData[0];
// //             msg.value = (int16_t)((RxData[1] << 8) | RxData[2]);
            
// //             uint64_t queueData;
// //             memcpy(&queueData, &msg, sizeof(CommandMsg_t));

// //             if (CanMotorCmdQueueHandle != NULL)
// //             {
// //                 osMessageQueuePut(CanMotorCmdQueueHandle, &queueData, 0, 0);
// //             }
// //         }
// //     }
// // }

// void CanService_SendFeedback(CanFeedback_t *feedback)
// {
//     /*
//     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    
//     CAN_TxHeaderTypeDef txHeader;
//     uint8_t txData[8];
//     uint32_t txMailbox;
    
//     txHeader.StdId = 0x101; // 电机反馈消息 ID
//     txHeader.RTR = CAN_RTR_DATA;
//     txHeader.DLC = 5; // 数据长度（1字节状态 + 2字节速度 + 2字节PWM）
    
//     txData[0] = feedback->motor_state;
//     *(int16_t*)(txData + 1) = feedback->current_logic_speed;
//     *(int16_t*)(txData + 3) = feedback->pwm_output;
    
//     if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
//     {
//         // 发送失败处理（可记录日志）
        
//     }
//     */
// }

// /*  <-- 从这里开始删除
//  * @brief 发送CAN反馈消息
//  * 
//  * @param type 
//  * @param value 
// void CanService_SendFeedback(CommandType_t type, int16_t value)
// {
    
//     CAN_TxHeaderTypeDef txHeader;
//     txHeader.StdId = 0x101; // 反馈ID
//     txHeader.ExtId = 0;
//     txHeader.IDE = CAN_ID_STD;
//     txHeader.RTR = CAN_RTR_DATA;
//     txHeader.DLC = 8;
//     txHeader.TransmitGlobalTime = DISABLE;

//     uint8_t txData[8] = {0};
//     txData[0] = (uint8_t)type;
//     txData[1] = (value >> 8) & 0xFF;
//     txData[2] = value & 0xFF;

//     uint32_t txMailbox;
//     if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
//     {
//         // 发送失败，闪烁PB4
//         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
//     }
    
// }
// */ // <-- 删除到这里结束