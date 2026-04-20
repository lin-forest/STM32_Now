/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

// #include "app_includes.h"
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
        uint32_t id = rxHeader.StdId;

        // 只处理已知 ID，其余丢弃
        if (id != CAN_MOTOR_TURN_CMD_STDID  && id != CAN_MOTOR_TURN_CMD_STATUS_STDID  &&
            id != CAN_MOTOR_POWER_CMD_STDID && id != CAN_MOTOR_POWER_CMD_STATUS_STDID &&
            id != CAN_CMD_STOP_STDID && id != CAN_CMD_TURN_STDID && id != CAN_CMD_POWER_STDID)
        {
            return;
        }

        CommandMsg_t cmdMsg;
        cmdMsg.type     = CMD_NONE;
        cmdMsg.value    = 0;

        // 根据 CAN ID 决定目标电机
        if (id == CAN_MOTOR_TURN_CMD_STDID || id == CAN_MOTOR_TURN_CMD_STATUS_STDID || id == CAN_CMD_TURN_STDID)
        {
            cmdMsg.motor_id = 0;          // 转向电机（电机0）
        }
        else if (id == CAN_MOTOR_POWER_CMD_STDID || id == CAN_MOTOR_POWER_CMD_STATUS_STDID || id == CAN_CMD_POWER_STDID)
        {
            cmdMsg.motor_id = 1;          // 动力电机（电机1）
        }
        else  // CAN_CMD_STOP_STDID / CAN_CMD_TURN_STDID / CAN_CMD_POWER_STDID
        {
            cmdMsg.motor_id = 0xFF;       // 广播：两个电机都执行
        }

        // 协议解析
        switch (rxData[0])
        {
            case CAN_CMD_SET_SPEED_T2:
                cmdMsg.type  = CAN_CMD_SET_SPEED;
                cmdMsg.value = (int16_t)rxData[1];
                break;

            case CAN_CMD_SET_SPEED:
                cmdMsg.type  = CAN_CMD_SET_SPEED;
                cmdMsg.value = (int8_t)rxData[CAN_DATA_INDEX_SPEED];
                break;

            case CAN_CMD_STOP:
                cmdMsg.type = CAN_CMD_STOP;
                break;

            case CAN_CMD_QUERY_STATUS:
                if (id == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                    id == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                    cmdMsg.type = CMD_QUERY_STATUS;
                break;

            case CAN_CMD_LOG_START:
                if (id == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                    id == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                    cmdMsg.type = CMD_LOG_START;
                break;

            case CAN_CMD_LOG_STOP:
                if (id == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                    id == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                    cmdMsg.type = CMD_LOG_STOP;
                break;

            default:
                return;
        }

        if (cmdMsg.type != CMD_NONE)
        {
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
    // 配置CAN过滤器，放行所有消息，由软件中断回调根据 ID 列表进行过滤
    sFilterConfig.FilterBank = CAN_FILTER_BANK;
    sFilterConfig.FilterMode = CAN_FILTER_MODE;
    sFilterConfig.FilterScale = CAN_FILTER_SCALE;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000; // 掩码设为0，接收所有 StdID
    sFilterConfig.FilterMaskIdLow = 0x0000;
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

