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

#include "app_task.h"
#include "app_globals.h"
#include "command.h"
#include "app_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for MotorControl_Ta */
osThreadId_t MotorControl_TaHandle;
uint32_t MotorControl_TaBuffer[ 64 ];
osStaticThreadDef_t MotorControl_TaControlBlock;
const osThreadAttr_t MotorControl_Ta_attributes = {
  .name = "MotorControl_Ta",
  .cb_mem = &MotorControl_TaControlBlock,
  .cb_size = sizeof(MotorControl_TaControlBlock),
  .stack_mem = &MotorControl_TaBuffer[0],
  .stack_size = sizeof(MotorControl_TaBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Encoder_Ta */
osThreadId_t Encoder_TaHandle;
uint32_t Encoder_TaBuffer[ 64 ];
osStaticThreadDef_t Encoder_TaControlBlock;
const osThreadAttr_t Encoder_Ta_attributes = {
  .name = "Encoder_Ta",
  .cb_mem = &Encoder_TaControlBlock,
  .cb_size = sizeof(Encoder_TaControlBlock),
  .stack_mem = &Encoder_TaBuffer[0],
  .stack_size = sizeof(Encoder_TaBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for Logger_Ta */
osThreadId_t Logger_TaHandle;
uint32_t Logger_TaBuffer[ 256 ];
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
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Ack_Ta */
osThreadId_t Ack_TaHandle;
uint32_t Ack_TaBuffer[ 256 ];
osStaticThreadDef_t Ack_TaControlBlock;
const osThreadAttr_t Ack_Ta_attributes = {
  .name = "Ack_Ta",
  .cb_mem = &Ack_TaControlBlock,
  .cb_size = sizeof(Ack_TaControlBlock),
  .stack_mem = &Ack_TaBuffer[0],
  .stack_size = sizeof(Ack_TaBuffer),
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for MotorControl1_T */
osThreadId_t MotorControl1_THandle;
uint32_t MotorControl1_TBuffer[ 64 ];
osStaticThreadDef_t MotorControl1_TControlBlock;
const osThreadAttr_t MotorControl1_T_attributes = {
  .name = "MotorControl1_T",
  .cb_mem = &MotorControl1_TControlBlock,
  .cb_size = sizeof(MotorControl1_TControlBlock),
  .stack_mem = &MotorControl1_TBuffer[0],
  .stack_size = sizeof(MotorControl1_TBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Encoder1_T */
osThreadId_t Encoder1_THandle;
uint32_t Encoder1_TBuffer[ 64 ];
osStaticThreadDef_t Encoder1_TControlBlock;
const osThreadAttr_t Encoder1_T_attributes = {
  .name = "Encoder1_T",
  .cb_mem = &Encoder1_TControlBlock,
  .cb_size = sizeof(Encoder1_TControlBlock),
  .stack_mem = &Encoder1_TBuffer[0],
  .stack_size = sizeof(Encoder1_TBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal2,
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
/* Definitions for MotorQueue1 */
osMessageQueueId_t MotorQueue1Handle;
const osMessageQueueAttr_t MotorQueue1_attributes = {
  .name = "MotorQueue1"
};
/* Definitions for motor0_mutex */
osMutexId_t motor0_mutexHandle;
const osMutexAttr_t motor0_mutex_attributes = {
  .name = "motor0_mutex"
};
/* Definitions for motor1_mutex */
osMutexId_t motor1_mutexHandle;
const osMutexAttr_t motor1_mutex_attributes = {
  .name = "motor1_mutex"
};
/* Definitions for uart_rx_semaphore */
osSemaphoreId_t uart_rx_semaphoreHandle;
osStaticSemaphoreDef_t uart_rx_semaphoreControlBlock;
const osSemaphoreAttr_t uart_rx_semaphore_attributes = {
  .name = "uart_rx_semaphore",
  .cb_mem = &uart_rx_semaphoreControlBlock,
  .cb_size = sizeof(uart_rx_semaphoreControlBlock),
};
/* Definitions for can_rx_semaphore */
osSemaphoreId_t can_rx_semaphoreHandle;
osStaticSemaphoreDef_t can_rx_semaphoreControlBlock;
const osSemaphoreAttr_t can_rx_semaphore_attributes = {
  .name = "can_rx_semaphore",
  .cb_mem = &can_rx_semaphoreControlBlock,
  .cb_size = sizeof(can_rx_semaphoreControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Heartbeat(void *argument);
void Start_MotorControl(void *argument);
void Start_Encoder(void *argument);
void Start_Logger(void *argument);
void Start_Command(void *argument);
void Start_Ack(void *argument);
void Start_MotorControl1_T(void *argument);
void Start_Encoder1_T(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of motor0_mutex */
  motor0_mutexHandle = osMutexNew(&motor0_mutex_attributes);

  /* creation of motor1_mutex */
  motor1_mutexHandle = osMutexNew(&motor1_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart_rx_semaphore */
  uart_rx_semaphoreHandle = osSemaphoreNew(1, 1, &uart_rx_semaphore_attributes);

  /* creation of can_rx_semaphore */
  can_rx_semaphoreHandle = osSemaphoreNew(1, 1, &can_rx_semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (72, sizeof(CommandMsg_t), &CommandQueue_attributes);

  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (72, sizeof(CommandMsg_t), &MotorQueue_attributes);

  /* creation of AckQueue */
  AckQueueHandle = osMessageQueueNew (64, sizeof(AckMsg_t), &AckQueue_attributes);

  /* creation of MotorQueue1 */
  MotorQueue1Handle = osMessageQueueNew (72, sizeof(CommandMsg_t), &MotorQueue1_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat_Ta */
  Heartbeat_TaHandle = osThreadNew(Start_Heartbeat, NULL, &Heartbeat_Ta_attributes);

  /* creation of MotorControl_Ta */
  MotorControl_TaHandle = osThreadNew(Start_MotorControl, NULL, &MotorControl_Ta_attributes);

  /* creation of Encoder_Ta */
  Encoder_TaHandle = osThreadNew(Start_Encoder, NULL, &Encoder_Ta_attributes);

  /* creation of Logger_Ta */
  Logger_TaHandle = osThreadNew(Start_Logger, NULL, &Logger_Ta_attributes);

  /* creation of Command_Ta */
  Command_TaHandle = osThreadNew(Start_Command, NULL, &Command_Ta_attributes);

  /* creation of Ack_Ta */
  Ack_TaHandle = osThreadNew(Start_Ack, NULL, &Ack_Ta_attributes);

  /* creation of MotorControl1_T */
  MotorControl1_THandle = osThreadNew(Start_MotorControl1_T, (void*) &g_motors[1], &MotorControl1_T_attributes);

  /* creation of Encoder1_T */
  Encoder1_THandle = osThreadNew(Start_Encoder1_T, (void*) &g_motors[1], &Encoder1_T_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Heartbeat */
/**
  * @brief  Function implementing the Heartbeat_Ta thread.
  * @param  argument: Not used
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

/* USER CODE BEGIN Header_Start_MotorControl */
/**
* @brief Function implementing the MotorControl_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MotorControl */
void Start_MotorControl(void *argument)
{
  /* USER CODE BEGIN Start_MotorControl */
  /* Infinite loop */
  
  #if ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612
  TB6612_DC_Task(argument);
  #elif ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236
  AT8236_DC_Task(argument);

  #else

  #error "No valid motor driver selected! Please check ACTIVE_MOTOR_DRIVER in app_config.h"

  #endif
  
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

/* USER CODE BEGIN Header_Start_MotorControl1_T */
/**
* @brief Function implementing the MotorControl1_T thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MotorControl1_T */
void Start_MotorControl1_T(void *argument)
{
  /* USER CODE BEGIN Start_MotorControl1_T */
  #if ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612
  TB6612_DC_Task(argument);
  #elif ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236
  AT8236_DC_Task(argument);
  #else
  #error "No valid motor driver selected! Please check ACTIVE_MOTOR_DRIVER in app_config.h"
  #endif
  /* USER CODE END Start_MotorControl1_T */
}

/* USER CODE BEGIN Header_Start_Encoder1_T */
/**
* @brief Function implementing the Encoder1_T thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Encoder1_T */
void Start_Encoder1_T(void *argument)
{
  /* USER CODE BEGIN Start_Encoder1_T */
  Encoder_Task(argument);
  /* USER CODE END Start_Encoder1_T */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

