


#include "app_includes.h"

/**
 * @brief  UART到CAN转换任务的实现函数
 * @param  argument: 未使用
 * @retval None
 */
void UartToCan_Task_Run(void *argument)
{
  /* USER CODE BEGIN UartToCan_Task_Run */
  App_CAN_Message_t tx_can_msg;

  /* Infinite loop */
  for(;;)
  {
    // 1. 从uartToCanQueue队列中等待并接收数据
    if (osMessageQueueGet(uartToCanQueueHandle, &tx_can_msg, NULL, osWaitForever) == osOK)
    {
      // 2. 成功接收到数据，准备通过CAN总线发送
      //    (此处可以添加报文的二次处理逻辑)

      // 3. 调用CAN发送函数
      // HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_can_msg.data, &tx_mailbox);
    }
  }
  /* USER CODE END UartToCan_Task_Run */
}

/**
 * @brief  CAN接收处理任务的实现函数
 * @param  argument: 未使用
 * @retval None
 */
void CanRxProcess_Task_Run(void *argument)
{
  /* USER CODE BEGIN CanRxProcess_Task_Run */
  App_CAN_Message_t rx_can_msg;

  /* Infinite loop */
  for(;;)
  {
    // 1. 从canRxQueue队列中等待并接收数据
    if (osMessageQueueGet(canRxQueueHandle, &rx_can_msg, NULL, osWaitForever) == osOK)
    {
      // 2. 成功接收到数据，准备通过UART发送给上位机
      //    (此处可以添加数据解析和格式化逻辑)
      
      // 3. 调用UART发送函数
      // HAL_UART_Transmit(&huart1, (uint8_t*)&rx_can_msg, sizeof(rx_can_msg), 0xFFFF);
    }
  }
  /* USER CODE END CanRxProcess_Task_Run */
}

/**

 * @brief  心跳任务的实现函数
 * @param  argument: 未使用
 * @retval None
 */
void Heartbeat_Task_Run(void *argument)
{
  /* USER CODE BEGIN Heartbeat_Task_Run */
  /* Infinite loop */
  for(;;)
  {
    // 翻转PC13引脚的电平 (通常是板载LED)
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    // 任务延时300ms
    osDelay(300);
  }
  /* USER CODE END Heartbeat_Task_Run */
}