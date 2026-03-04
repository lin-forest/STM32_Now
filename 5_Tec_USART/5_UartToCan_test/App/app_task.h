#ifndef APP_TASK_H
#define APP_TASK_H

/* Includes ------------------------------------------------------------------*/
// #include "app_includes.h"

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  UART到CAN转换任务的入口函数
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void StartUartToCanTask(void *argument);

/**
 * @brief  CAN接收处理任务的入口函数
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void StartCanRxProcessTask(void *argument);

/**
 * @brief  心跳任务的入口函数
 * @param  argument: 任务参数 (未使用)
 * @retval None
 */
void Start_Heartbeat(void *argument);


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


#endif /* APP_TASK_H */