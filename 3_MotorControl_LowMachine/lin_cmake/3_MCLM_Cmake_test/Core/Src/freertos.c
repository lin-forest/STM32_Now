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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "app_task.h"

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
/* Definitions for MotorControl_Ta */
osThreadId_t MotorControl_TaHandle;
uint32_t MotorControl_TBuffer[ 128 ];
osStaticThreadDef_t MotorControl_TControlBlock;
const osThreadAttr_t MotorControl_Ta_attributes = {
  .name = "MotorControl_Ta",
  .cb_mem = &MotorControl_TControlBlock,
  .cb_size = sizeof(MotorControl_TControlBlock),
  .stack_mem = &MotorControl_TBuffer[0],
  .stack_size = sizeof(MotorControl_TBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Encoder_Ta */
osThreadId_t Encoder_TaHandle;
uint32_t Encoder_TaskBuffer[ 128 ];
osStaticThreadDef_t Encoder_TaskControlBlock;
const osThreadAttr_t Encoder_Ta_attributes = {
  .name = "Encoder_Ta",
  .cb_mem = &Encoder_TaskControlBlock,
  .cb_size = sizeof(Encoder_TaskControlBlock),
  .stack_mem = &Encoder_TaskBuffer[0],
  .stack_size = sizeof(Encoder_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Logger_Ta */
osThreadId_t Logger_TaHandle;
uint32_t Logger_TaBuffer[ 384 ];
osStaticThreadDef_t Logger_TaControlBlock;
const osThreadAttr_t Logger_Ta_attributes = {
  .name = "Logger_Ta",
  .cb_mem = &Logger_TaControlBlock,
  .cb_size = sizeof(Logger_TaControlBlock),
  .stack_mem = &Logger_TaBuffer[0],
  .stack_size = sizeof(Logger_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Command_Ta */
osThreadId_t Command_TaHandle;
uint32_t Command_TaBuffer[ 128 ];
osStaticThreadDef_t Command_TaControlBlock;
const osThreadAttr_t Command_Ta_attributes = {
  .name = "Command_Ta",
  .cb_mem = &Command_TaControlBlock,
  .cb_size = sizeof(Command_TaControlBlock),
  .stack_mem = &Command_TaBuffer[0],
  .stack_size = sizeof(Command_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Heartbeat_Ta */
osThreadId_t Heartbeat_TaHandle;
uint32_t Heartbeat_TaBuffer[ 128 ];
osStaticThreadDef_t Heartbeat_TaControlBlock;
const osThreadAttr_t Heartbeat_Ta_attributes = {
  .name = "Heartbeat_Ta",
  .cb_mem = &Heartbeat_TaControlBlock,
  .cb_size = sizeof(Heartbeat_TaControlBlock),
  .stack_mem = &Heartbeat_TaBuffer[0],
  .stack_size = sizeof(Heartbeat_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ack_Ta */
osThreadId_t Ack_TaHandle;
uint32_t Ack_TaBuffer[ 128 ];
osStaticThreadDef_t Ack_TaControlBlock;
const osThreadAttr_t Ack_Ta_attributes = {
  .name = "Ack_Ta",
  .cb_mem = &Ack_TaControlBlock,
  .cb_size = sizeof(Ack_TaControlBlock),
  .stack_mem = &Ack_TaBuffer[0],
  .stack_size = sizeof(Ack_TaBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};
/* Definitions for AckQueue */
osMessageQueueId_t AckQueueHandle;
const osMessageQueueAttr_t AckQueue_attributes = {
  .name = "AckQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_MotorControl(void *argument);
void Start_Encoder(void *argument);
void Start_Logger(void *argument);
void Start_Command(void *argument);
void Start_Heartbeat(void *argument);
void Start_Ack(void *argument);

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
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (64, sizeof(uint64_t), &CommandQueue_attributes);

  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (72, sizeof(uint64_t), &MotorQueue_attributes);

  /* creation of AckQueue */
  AckQueueHandle = osMessageQueueNew (64, sizeof(uint64_t), &AckQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorControl_Ta */
  MotorControl_TaHandle = osThreadNew(Start_MotorControl, NULL, &MotorControl_Ta_attributes);

  /* creation of Encoder_Ta */
  Encoder_TaHandle = osThreadNew(Start_Encoder, NULL, &Encoder_Ta_attributes);

  /* creation of Logger_Ta */
  Logger_TaHandle = osThreadNew(Start_Logger, NULL, &Logger_Ta_attributes);

  /* creation of Command_Ta */
  Command_TaHandle = osThreadNew(Start_Command, NULL, &Command_Ta_attributes);

  /* creation of Heartbeat_Ta */
  Heartbeat_TaHandle = osThreadNew(Start_Heartbeat, NULL, &Heartbeat_Ta_attributes);

  /* creation of Ack_Ta */
  Ack_TaHandle = osThreadNew(Start_Ack, NULL, &Ack_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_MotorControl */
/**
  * @brief  Function implementing the MotorControl_Ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_MotorControl */
void Start_MotorControl(void *argument)
{
  /* USER CODE BEGIN Start_MotorControl */
  /* Infinite loop */

  MotorControl_Task(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_MotorControl */
}

/* USER CODE BEGIN Header_Start_Encoder */
/**
* @brief Function implementing the Encoder_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Encoder */
void Start_Encoder(void *argument)
{
  /* USER CODE BEGIN Start_Encoder */
  /* Infinite loop */

  Encoder_Task(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_Encoder */
}

/* USER CODE BEGIN Header_Start_Logger */
/**
* @brief Function implementing the Logger_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Logger */
void Start_Logger(void *argument)
{
  /* USER CODE BEGIN Start_Logger */
  /* Infinite loop */

  Logger_Task(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_Logger */
}

/* USER CODE BEGIN Header_Start_Command */
/**
* @brief Function implementing the Command_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Command */
void Start_Command(void *argument)
{
  /* USER CODE BEGIN Start_Command */
  /* Infinite loop */

  Command_Task(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_Command */
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

  Heartbeat_Task(argument);

  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_Heartbeat */
}

/* USER CODE BEGIN Header_Start_Ack */
/**
* @brief Function implementing the Ack_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Ack */
void Start_Ack(void *argument)
{
  /* USER CODE BEGIN Start_Ack */
  /* Infinite loop */

  Ack_Task(argument);
  
  // for(;;)
  // {
  //   osDelay(1);
  // }
  /* USER CODE END Start_Ack */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

