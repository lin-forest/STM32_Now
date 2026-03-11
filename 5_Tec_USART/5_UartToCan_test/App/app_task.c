


#include "app_includes.h"
#include "usart.h"

/**
 * @brief 引用在usart.c中定义的环形缓冲区
 */
extern RingBuffer_t uart1_rx_buffer;

/**
 * @brief  UART到CAN转换任务的实现函数
 * @param  argument: 未使用
 * @retval None
 */
void UartToCan_Task_Run(void *argument)
{
  /* USER CODE BEGIN UartToCan_Task_Run */
  App_UART_Message_t uart_msg;
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;
  char dbg_buffer[128]; // 用于诊断打印的缓冲区

  tx_header.RTR = CAN_RTR_DATA;       // 我们只处理数据帧
  tx_header.TransmitGlobalTime = DISABLE;

  /* Infinite loop */
  for(;;)
  {
    // 1. 从uartToCanQueue队列中等待并接收数据
    if (osMessageQueueGet(uartToCanQueueHandle, &uart_msg, NULL, osWaitForever) == osOK)
    {
      // 2. [诊断探针#1] 打印收到的消息，确认协议解析任务工作正常
      int offset = sprintf(dbg_buffer, "UART->CAN | RX_MSG | ID: 0x%lX, DLC: %d. Sending...\r\n", uart_msg.id, uart_msg.len);
      HAL_UART_Transmit(&huart1, (uint8_t*)dbg_buffer, offset, 0xFFFF);

      // 3. [增强逻辑] 根据ID大小，自动判断是标准帧还是扩展帧
      if (uart_msg.id > 0x7FF) { // CAN ID大于11位，为扩展帧
          tx_header.IDE = CAN_ID_EXT;
          tx_header.ExtId = uart_msg.id;
      } else { // 标准帧
          tx_header.IDE = CAN_ID_STD;
          tx_header.StdId = uart_msg.id;
      }
      
      tx_header.DLC = uart_msg.len;   // 设置数据长度
      
      // 4. 调用CAN发送函数，并检查其返回值
      HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);

      // 5. [诊断探针#2] 如果CAN发送失败，打印出错误状态
      if (tx_status != HAL_OK)
      {
          offset = sprintf(dbg_buffer, "UART->CAN | TX_FAIL | Status: %d\r\n", tx_status);
          HAL_UART_Transmit(&huart1, (uint8_t*)dbg_buffer, offset, 0xFFFF);
      }
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
  char tx_buffer[128]; // 用于构建发送字符串的缓冲区

  /* Infinite loop */
  for(;;)
  {
    // 1. 从canRxQueue队列中等待并接收数据
    if (osMessageQueueGet(canRxQueueHandle, &rx_can_msg, NULL, osWaitForever) == osOK)
    {
      // 2. 成功接收到数据，将其格式化为可读字符串
      int offset = sprintf(tx_buffer, "CAN RX | ID: 0x%03lX | DLC: %d | Data: ", rx_can_msg.id, rx_can_msg.len);
      
      for (int i = 0; i < rx_can_msg.len; i++)
      {
        offset += sprintf(tx_buffer + offset, "%02X ", rx_can_msg.data[i]);
      }
      
      offset += sprintf(tx_buffer + offset, "\r\n");

      // 3. 调用UART发送函数，将格式化后的字符串发送出去
      HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, offset, 0xFFFF);
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

/**
 * @brief  协议解析任务的实现函数
 * @param  argument: 未使用
 * @retval None
 */
void ProtocolParser_Task_Run(void *argument)
{
    // 定义解析状态枚举
    typedef enum {
        STATE_WAIT_SOF,  // 等待帧头 (Start of Frame)
        STATE_WAIT_CMD,  // 等待指令ID
        STATE_WAIT_ID,   // 等待4字节的CAN ID
        STATE_WAIT_LEN,  // 等待数据长度
        STATE_WAIT_DATA  // 等待数据负载
    } ParserState_t;

    ParserState_t state = STATE_WAIT_SOF;
    App_UART_Message_t current_msg;
    uint8_t byte_received;
    uint8_t data_idx = 0;
    uint8_t id_byte_count = 0;

    for(;;)
    {
        // 1. 尝试从环形缓冲区获取一个字节
        if (ring_buffer_get(&uart1_rx_buffer, &byte_received))
        {
            // 2. 根据当前状态处理接收到的字节 (状态机)
            switch (state)
            {
                case STATE_WAIT_SOF:
                    if (byte_received == FRAME_SOF) {
                        // 检测到帧头，清零整个消息结构体，准备接收新消息
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        state = STATE_WAIT_CMD;
                    }
                    break;

                case STATE_WAIT_CMD:
                    current_msg.cmd = byte_received;
                    id_byte_count = 0;
                    state = STATE_WAIT_ID;
                    break;

                case STATE_WAIT_ID:
                    // 按照小端模式(Little-Endian)拼接4字节ID
                    current_msg.id |= (uint32_t)byte_received << (8 * id_byte_count);
                    id_byte_count++;
                    if (id_byte_count >= 4) {
                        state = STATE_WAIT_LEN;
                    }
                    break;

                case STATE_WAIT_LEN:
                    if (byte_received <= 8) { // 检查数据长度是否有效
                        current_msg.len = byte_received;
                        data_idx = 0;
                        if (current_msg.len > 0) {
                            state = STATE_WAIT_DATA;
                        } else {
                            // 如果数据长度为0, 报文接收完成
                            osMessageQueuePut(uartToCanQueueHandle, &current_msg, 0, 0);
                            state = STATE_WAIT_SOF;
                        }
                    } else {
                        // 长度无效, 丢弃报文, 重置状态机
                        state = STATE_WAIT_SOF;
                    }
                    break;

                case STATE_WAIT_DATA:
                    current_msg.data[data_idx++] = byte_received;
                    if (data_idx >= current_msg.len) {
                        // 所有数据字节接收完毕, 报文接收完成
                        osMessageQueuePut(uartToCanQueueHandle, &current_msg, 0, 0);
                        state = STATE_WAIT_SOF;
                    }
                    break;
            }
        }
        else
        {
            // 3. 如果缓冲区为空, 任务休眠1ms, 避免忙等, 让出CPU
            osDelay(1);
        }
    }
}