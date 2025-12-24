#include "can.h"
#include "app_includes.h"
// #include "can_service.h" // Add this line to include its own header

// QueueHandle_t CanMotorCmdQueueHandle;

void CanService_Init(void)
{
    MX_CAN_Init(); // Changed from MX_CAN1_Init
//     CanMotorCmdQueueHandle = xQueueCreate(CAN_CMD_QUEUE_SIZE, sizeof(CommandMsg_t));
    
    // 配置 CAN 滤波（仅接收电机相关消息 ID，示例 ID=0x100）
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE; // Changed from FilterActivate
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) // Changed from &hcan1
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) // Changed from &hcan1
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // Changed from &hcan1
    {
        Error_Handler();
    }
}

// CAN 接收中断回调（将 CAN 指令转换为现有 CommandMsg_t 入队）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_handle)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    CommandMsg_t cmdMsg; // 使用统一的 CommandMsg_t

    if (HAL_CAN_GetRxMessage(hcan_handle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }

    // 示例：解析 CAN 消息并转换为 CommandMsg_t
    // 这里需要根据实际的 CAN 协议进行解析
    // 假设 CAN ID 决定命令类型，数据决定值
    if (RxHeader.StdId == 0x100) // 假设 CAN ID 0x100 为设置速度命令
    {
        cmdMsg.type = CAN_CMD_SET_SPEED;
        cmdMsg.value = (int16_t)((RxData[0] << 8) | RxData[1]); // 假设速度值在数据的前两个字节
    }
    else if (RxHeader.StdId == 0x101) // 假设 CAN ID 0x101 为停止命令
    {
        cmdMsg.type = CAN_CMD_STOP;
        cmdMsg.value = 0;
    }
    else
    {
        cmdMsg.type = CMD_NONE; // 未知命令
    }

    if (cmdMsg.type != CMD_NONE)
    {
        xQueueSendFromISR(CanMotorCmdQueueHandle, &cmdMsg, NULL);
    }
}

void CanService_SendFeedback(CanFeedback_t *feedback)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;
    
    txHeader.StdId = 0x101; // 电机反馈消息 ID
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 5; // 数据长度（1字节状态 + 2字节速度 + 2字节PWM）
    
    txData[0] = feedback->motor_state;
    *(int16_t*)(txData + 1) = feedback->current_logic_speed;
    *(int16_t*)(txData + 3) = feedback->pwm_output;
    
    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
    {
        // 发送失败处理（可记录日志）
    }
}