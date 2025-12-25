/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "app_task.h"  // 添加这一行

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for Heart_Ta */
osThreadId_t Heart_TaHandle;
uint32_t Heart_TaBuffer[ 128 ];
osStaticThreadDef_t Heart_TaControlBlock;
const osThreadAttr_t Heart_Ta_attributes = {
  .name = "Heart_Ta",
  .cb_mem = &Heart_TaControlBlock,
  .cb_size = sizeof(Heart_TaControlBlock),
  .stack_mem = &Heart_TaBuffer[0],
  .stack_size = sizeof(Heart_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartRx_Ta */
osThreadId_t UartRx_TaHandle;
uint32_t UartRx_TaBuffer[ 256 ];
osStaticThreadDef_t UartRx_TaControlBlock;
const osThreadAttr_t UartRx_Ta_attributes = {
  .name = "UartRx_Ta",
  .cb_mem = &UartRx_TaControlBlock,
  .cb_size = sizeof(UartRx_TaControlBlock),
  .stack_mem = &UartRx_TaBuffer[0],
  .stack_size = sizeof(UartRx_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanTx_Ta */
osThreadId_t CanTx_TaHandle;
uint32_t CanTx_TaBuffer[ 256 ];
osStaticThreadDef_t CanTx_TaControlBlock;
const osThreadAttr_t CanTx_Ta_attributes = {
  .name = "CanTx_Ta",
  .cb_mem = &CanTx_TaControlBlock,
  .cb_size = sizeof(CanTx_TaControlBlock),
  .stack_mem = &CanTx_TaBuffer[0],
  .stack_size = sizeof(CanTx_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartRxQueue */
osMessageQueueId_t UartRxQueueHandle;
const osMessageQueueAttr_t UartRxQueue_attributes = {
  .name = "UartRxQueue"
};
/* Definitions for CanCmdQueue */
osMessageQueueId_t CanCmdQueueHandle;
const osMessageQueueAttr_t CanCmdQueue_attributes = {
  .name = "CanCmdQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Heart_Ta(void *argument);
void Start_UartRx_Ta(void *argument);
void Start_CanTx_Ta(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UartRxQueue */
  UartRxQueueHandle = osMessageQueueNew (64, sizeof(uint64_t), &UartRxQueue_attributes);

  /* creation of CanCmdQueue */
  CanCmdQueueHandle = osMessageQueueNew (64, sizeof(uint64_t), &CanCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heart_Ta */
  Heart_TaHandle = osThreadNew(Start_Heart_Ta, NULL, &Heart_Ta_attributes);

  /* creation of UartRx_Ta */
  UartRx_TaHandle = osThreadNew(Start_UartRx_Ta, NULL, &UartRx_Ta_attributes);

  /* creation of CanTx_Ta */
  CanTx_TaHandle = osThreadNew(Start_CanTx_Ta, NULL, &CanTx_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Heart_Ta */
/**
  * @brief  Function implementing the Heart_Ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Heart_Ta */
void Start_Heart_Ta(void *argument)
{
  /* USER CODE BEGIN Start_Heart_Ta */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
    osDelay(100);
  }
  /* USER CODE END Start_Heart_Ta */
}

/* USER CODE BEGIN Header_Start_UartRx_Ta */
/**
* @brief Function implementing the UartRx_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UartRx_Ta */
void Start_UartRx_Ta(void *argument)
{
  /* USER CODE BEGIN Start_UartRx_Ta */
  /* Infinite loop */

  UartRxTask(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_UartRx_Ta */
}

/* USER CODE BEGIN Header_Start_CanTx_Ta */
/**
* @brief Function implementing the CanTx_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CanTx_Ta */
void Start_CanTx_Ta(void *argument)
{
  /* USER CODE BEGIN Start_CanTx_Ta */
  /* Infinite loop */

  CanTxTask(argument);
  
  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_CanTx_Ta */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */