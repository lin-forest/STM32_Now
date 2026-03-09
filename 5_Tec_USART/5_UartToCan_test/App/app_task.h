#ifndef APP_TASK_H
#define APP_TASK_H

/* Includes ------------------------------------------------------------------*/
// #include "app_includes.h"

/* Public function prototypes ------------------------------------------------*/

/* Task Implementation Functions -------------------------------------------*/

/**
 * @brief  UART到CAN转换任务的实现函数 (包含无限循环)
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void UartToCan_Task_Run(void *argument);

/**
 * @brief  CAN接收处理任务的实现函数 (包含无限循环)
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void CanRxProcess_Task_Run(void *argument);

/**
 * @brief  心跳任务的实现函数 (包含无限循环)
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void Heartbeat_Task_Run(void *argument);

/**
 * @brief  协议解析任务的实现函数 (包含无限循环)
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
// void ProtocolParser_Task_Run(void *argument);


#endif /* APP_TASK_H */