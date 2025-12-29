#include "can.h"
#include "app_includes.h"
#include "stm32f1xx_hal_gpio.h"
#include "can_service.h" // Add this line to include its own header

// CAN 接收中断回调（将 CAN 指令转换为现有 CommandMsg_t 入队）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        // 确认ID (虽然过滤器已经保证了)
        if (rxHeader.StdId == 0x40) 
        {
            // 3. 构建并发送 CommandMsg_t 结构体
            CommandMsg_t cmdMsg;
            cmdMsg.type = CAN_CMD_SET_SPEED;
            // 从CAN数据帧的第二个字节获取速度值
            // 注意：这里假设速度值是一个 signed 8-bit integer (int8_t)
            // 如果是 unsigned，请使用 uint8_t
            cmdMsg.value = (int8_t)rxData[1]; 

            // 使用 CMSIS API 从中断发送队列
            osMessageQueuePut(CanMotorCmdQueueHandle, &cmdMsg, 0U, 0U);
        }
    }
}
