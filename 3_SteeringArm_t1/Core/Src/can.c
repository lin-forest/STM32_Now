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

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
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

  /*
   * 硬件滤波 — 32-bit mask mode，精确匹配本机关心的 CAN ID
   *
   *    Bank 0: 0x130 + 0x131 (mask=0x7FE, 忽略 bit0)
   *    Bank 1: 0x230          (mask=0x7FF, 精确匹配)
   *    Bank 2: 0x430          (mask=0x7FF, 精确匹配)
   *
   * 寄存器布局 (CAN_FxR1 / CAN_FxR2, 32-bit):
   *   [31:21] = STID[10:0], [2] = IDE, [1] = RTR
   */
  CAN_FilterTypeDef can_filter = {0};

  /* Bank 0: accept 0x130 and 0x131 */
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh        = (uint16_t)((0x130UL << 21) >> 16);
  can_filter.FilterIdLow         = (uint16_t)((0x130UL << 21) & 0xFFFF);
  can_filter.FilterMaskIdHigh    = (uint16_t)((0x7FEUL << 21) >> 16);
  can_filter.FilterMaskIdLow     = (uint16_t)(((0x7FEUL << 21) & 0xFFFF) | 0x0004);
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation    = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &can_filter);

  /* Bank 1: accept 0x230 */
  can_filter.FilterBank = 1;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterIdHigh        = (uint16_t)((0x230UL << 21) >> 16);
  can_filter.FilterIdLow         = (uint16_t)((0x230UL << 21) & 0xFFFF);
  can_filter.FilterMaskIdHigh    = (uint16_t)((0x7FFUL << 21) >> 16);
  can_filter.FilterMaskIdLow     = (uint16_t)(((0x7FFUL << 21) & 0xFFFF) | 0x0004);
  HAL_CAN_ConfigFilter(&hcan, &can_filter);

  /* Bank 2: accept 0x430 */
  can_filter.FilterBank = 2;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterIdHigh        = (uint16_t)((0x430UL << 21) >> 16);
  can_filter.FilterIdLow         = (uint16_t)((0x430UL << 21) & 0xFFFF);
  can_filter.FilterMaskIdHigh    = (uint16_t)((0x7FFUL << 21) >> 16);
  can_filter.FilterMaskIdLow     = (uint16_t)(((0x7FFUL << 21) & 0xFFFF) | 0x0004);
  HAL_CAN_ConfigFilter(&hcan, &can_filter);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR);

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
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

