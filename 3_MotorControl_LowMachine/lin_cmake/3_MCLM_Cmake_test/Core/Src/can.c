/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "cmsis_os.h" // 使用 CMSIS-OS API
#include "stm32f1xx_hal_gpio.h"
#include "string.h"
#include "stdint.h"
#include "command.h"
#include "app_config.h"

// 使用在 freertos.c 中定义的 CMSIS 句柄
extern osMessageQueueId_t CommandQueueHandle; 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        // [修改1] 移除 ID 检查，允许接收任意 ID 的消息
        // 只要数据长度不为0
        // if (rxHeader.DLC > 0)
        if(rxHeader.StdId == 0x124 || rxHeader.StdId == 0x101 ||
           rxHeader.StdId == 0x103 || rxHeader.StdId == 0x224)
        {
            CommandMsg_t cmdMsg;
            cmdMsg.type = CMD_NONE;
            cmdMsg.value = 0;

            // [修改2] 协议解析与映射
            // 根据接收到的第0个字节（命令字）进行判断
            switch (rxData[0])
            {
                // === 处理自定义数据帧: 11 22 33 44 55 66 77 00 ===
                case 0x11: 
                    // 将外部命令 0x11 映射为内部的 "设置速度" 指令
                    cmdMsg.type = CMD_SET_SPEED; 
                    
                    // 提取参数：假设第1个字节 (0x22) 是速度值
                    // 注意：0x22 = 34 (逻辑速度)
                    cmdMsg.value = (int16_t)rxData[1]; 
                    
                    // 如果您的速度值是16位的 (例如由 22 33 组成)，可以使用:
                    // cmdMsg.value = (int16_t)(rxData[1] | (rxData[2] << 8));
                    break;

                // === 兼容旧协议 ===
                case CAN_CMD_SET_SPEED:
                    cmdMsg.type = CAN_CMD_SET_SPEED;
                    cmdMsg.value = (int8_t)rxData[CAN_DATA_INDEX_SPEED];
                    break;
                
                case CAN_CMD_STOP:
                    cmdMsg.type = CAN_CMD_STOP;
                    break;

                case 0x01: // 查询命令（0x201帧）
                    if (rxHeader.StdId == 0x201)
                    {
                        cmdMsg.type = CMD_QUERY_STATUS;
                    }
                    break;

                default:
                    // 未知命令，直接返回，不发送到队列
                    return; 
            }

            // 3. 将解析后的命令发送到 CommandQueue
            // 保持原有的传递路径：CAN中断 -> CommandQueue -> CommandTask -> MotorTask
            osMessageQueuePut(CommandQueueHandle, &cmdMsg, 0U, 0U);
        }
    }
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
    CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    // 配置CAN过滤器，只接收ID为 0x7B 的消息
    sFilterConfig.FilterBank = CAN_FILTER_BANK;
    sFilterConfig.FilterMode = CAN_FILTER_MODE;
    sFilterConfig.FilterScale = CAN_FILTER_SCALE;
    sFilterConfig.FilterIdHigh = CAN_FILTER_ID_HIGH;
    sFilterConfig.FilterIdLow = CAN_FILTER_ID_LOW;
    sFilterConfig.FilterMaskIdHigh = CAN_FILTER_MASK_ID_HIGH;
    sFilterConfig.FilterMaskIdLow = CAN_FILTER_MASK_ID_LOW;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO;
    sFilterConfig.FilterActivation = CAN_FILTER_ACTIVATION;
    sFilterConfig.SlaveStartFilterBank = CAN_SLAVE_START_FILTER_BANK;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // 启动CAN
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // 使能接收中断
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
