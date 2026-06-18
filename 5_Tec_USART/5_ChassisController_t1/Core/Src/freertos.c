/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app_includes.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for UartToCan_Ta */
osThreadId_t UartToCan_TaHandle;
uint32_t uartToCanTaskBuffer[ 512 ];
osStaticThreadDef_t uartToCanTaskControlBlock;
const osThreadAttr_t UartToCan_Ta_attributes = {
  .name = "UartToCan_Ta",
  .cb_mem = &uartToCanTaskControlBlock,
  .cb_size = sizeof(uartToCanTaskControlBlock),
  .stack_mem = &uartToCanTaskBuffer[0],
  .stack_size = sizeof(uartToCanTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanRxProcess_Ta */
osThreadId_t CanRxProcess_TaHandle;
uint32_t canRxProcessTasBuffer[ 512 ];
osStaticThreadDef_t canRxProcessTasControlBlock;
const osThreadAttr_t CanRxProcess_Ta_attributes = {
  .name = "CanRxProcess_Ta",
  .cb_mem = &canRxProcessTasControlBlock,
  .cb_size = sizeof(canRxProcessTasControlBlock),
  .stack_mem = &canRxProcessTasBuffer[0],
  .stack_size = sizeof(canRxProcessTasBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Heartbeat_Ta */
osThreadId_t Heartbeat_TaHandle;
uint32_t Heartbeat_TaBuffer[ 64 ];
osStaticThreadDef_t Heartbeat_TaControlBlock;
const osThreadAttr_t Heartbeat_Ta_attributes = {
  .name = "Heartbeat_Ta",
  .cb_mem = &Heartbeat_TaControlBlock,
  .cb_size = sizeof(Heartbeat_TaControlBlock),
  .stack_mem = &Heartbeat_TaBuffer[0],
  .stack_size = sizeof(Heartbeat_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ProtocolParser_ */
osThreadId_t ProtocolParser_Handle;
uint32_t ProtocolParser_Buffer[ 256 ];
osStaticThreadDef_t ProtocolParser_ControlBlock;
const osThreadAttr_t ProtocolParser__attributes = {
  .name = "ProtocolParser_",
  .cb_mem = &ProtocolParser_ControlBlock,
  .cb_size = sizeof(ProtocolParser_ControlBlock),
  .stack_mem = &ProtocolParser_Buffer[0],
  .stack_size = sizeof(ProtocolParser_Buffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for CommandProcess_Ta */
osThreadId_t CommandProcess_TaHandle;
uint32_t CommandProcess_TaBuffer[ 256 ];
osStaticThreadDef_t CommandProcess_TaControlBlock;
const osThreadAttr_t CommandProcess_Ta_attributes = {
  .name = "CommandProcess_Ta",
  .cb_mem = &CommandProcess_TaControlBlock,
  .cb_size = sizeof(CommandProcess_TaControlBlock),
  .stack_mem = &CommandProcess_TaBuffer[0],
  .stack_size = sizeof(CommandProcess_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for uartToCanQueue */
osMessageQueueId_t uartToCanQueueHandle;
const osMessageQueueAttr_t uartToCanQueue_attributes = {
  .name = "uartToCanQueue"
};
/* Definitions for canTxQueue */
osMessageQueueId_t canTxQueueHandle;
const osMessageQueueAttr_t canTxQueue_attributes = {
  .name = "canTxQueue"
};
/* Definitions for uart1_tx_mutex */
osMutexId_t uart1_tx_mutexHandle;
osStaticMutexDef_t uart1_tx_mutexControlBlock;
const osMutexAttr_t uart1_tx_mutex_attributes = {
  .name = "uart1_tx_mutex",
  .cb_mem = &uart1_tx_mutexControlBlock,
  .cb_size = sizeof(uart1_tx_mutexControlBlock),
};
/* Definitions for uart1_tx_sem */
osSemaphoreId_t uart1_tx_semHandle;
osStaticSemaphoreDef_t uart1_tx_semControlBlock;
const osSemaphoreAttr_t uart1_tx_sem_attributes = {
  .name = "uart1_tx_sem",
  .cb_mem = &uart1_tx_semControlBlock,
  .cb_size = sizeof(uart1_tx_semControlBlock),
};
/* Definitions for uart1_rx_event */
osEventFlagsId_t uart1_rx_eventHandle;
osStaticEventGroupDef_t uart1_rx_eventControlBlock;
const osEventFlagsAttr_t uart1_rx_event_attributes = {
  .name = "uart1_rx_event",
  .cb_mem = &uart1_rx_eventControlBlock,
  .cb_size = sizeof(uart1_rx_eventControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_UartToCan(void *argument);
void Start_CanRxProcess(void *argument);
void Start_Heartbeat(void *argument);
void Start_ProtocolParser(void *argument);
void Start_CommandProcess(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)  
{
  /* 当检测到栈溢出时，程序会跳到这里 */
   /* 1. 强制翻转或点亮 LED（比如 PC13）作为视觉报警 */
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
   
   /* 2. 这里的 pcTaskName 指向溢出任务的名字，
         你可以在 Ozone 的 Watch 窗口查看这个变量 */
   (void)xTask;
   (void)pcTaskName;

   /* 3. 进入死循环，方便调试器接入 */
   while (1) 
   {
       // 在这一行打一个断点！
   }
   
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of uart1_tx_mutex */
  uart1_tx_mutexHandle = osMutexNew(&uart1_tx_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart1_tx_sem */
  uart1_tx_semHandle = osSemaphoreNew(1, 0, &uart1_tx_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (16, sizeof(App_CAN_Message_t), &canRxQueue_attributes);

  /* creation of uartToCanQueue */
  uartToCanQueueHandle = osMessageQueueNew (16, sizeof(App_UART_Message_t), &uartToCanQueue_attributes);

  /* creation of canTxQueue */
  canTxQueueHandle = osMessageQueueNew (16, sizeof(App_CAN_Message_t), &canTxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UartToCan_Ta */
  UartToCan_TaHandle = osThreadNew(Start_UartToCan, NULL, &UartToCan_Ta_attributes);

  /* creation of CanRxProcess_Ta */
  CanRxProcess_TaHandle = osThreadNew(Start_CanRxProcess, NULL, &CanRxProcess_Ta_attributes);

  /* creation of Heartbeat_Ta */
  Heartbeat_TaHandle = osThreadNew(Start_Heartbeat, NULL, &Heartbeat_Ta_attributes);

  /* creation of ProtocolParser_ */
  ProtocolParser_Handle = osThreadNew(Start_ProtocolParser, NULL, &ProtocolParser__attributes);

  /* creation of CommandProcess_Ta */
  CommandProcess_TaHandle = osThreadNew(Start_CommandProcess, NULL, &CommandProcess_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of uart1_rx_event */
  uart1_rx_eventHandle = osEventFlagsNew(&uart1_rx_event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_UartToCan */
/**
  * @brief  Function implementing the UartToCan_Ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_UartToCan */
void Start_UartToCan(void *argument)
{
  /* USER CODE BEGIN Start_UartToCan */
  /* Infinite loop */

  UartToCan_Task_Run(argument);
  /* USER CODE END Start_UartToCan */
}

/* USER CODE BEGIN Header_Start_CanRxProcess */
/**
* @brief Function implementing the CanRxProcess_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CanRxProcess */
void Start_CanRxProcess(void *argument)
{
  /* USER CODE BEGIN Start_CanRxProcess */
  /* Infinite loop */
  
  CanRxProcess_Task_Run(argument);
  /* USER CODE END Start_CanRxProcess */
}

/* USER CODE BEGIN Header_Start_Heartbeat */
/**
* @brief Function implementing the Heartbeat_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Heartbeat */
void Start_Heartbeat(void *argument)
{
  /* USER CODE BEGIN Start_Heartbeat */
  /* Infinite loop */
  
  Heartbeat_Task_Run(argument);
  /* USER CODE END Start_Heartbeat */
}

/* USER CODE BEGIN Header_Start_ProtocolParser */
/**
* @brief Function implementing the ProtocolParser_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ProtocolParser */
void Start_ProtocolParser(void *argument)
{
  /* USER CODE BEGIN Start_ProtocolParser */
  /* Infinite loop */

  ProtocolParser_Task_Run(argument);
  /* USER CODE END Start_ProtocolParser */
}

/* USER CODE BEGIN Header_Start_CommandProcess */
/**
* @brief Function implementing the CommandProcess_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CommandProcess */
void Start_CommandProcess(void *argument)
{
  /* USER CODE BEGIN Start_CommandProcess */
  CommandProcess_Task_Run(argument);
  /* USER CODE END Start_CommandProcess */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

