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
#include <stdio.h>
#include "app_config.h"
#include "app_globals.h"
#include "mt6701.h"
#include "servo.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"
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
ArmState_t g_arm_state = {0};
volatile uint8_t g_servo_active = 0;   /* 收到 CAN 命令后设为 1 */
volatile uint8_t g_system_locked = 0;  /* 430#03 锁定, 430#04 解锁 */
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_Rx_Ta */
osThreadId_t CAN_Rx_TaHandle;
uint32_t CAN_Rx_TaBuffer[ 256 ];
osStaticThreadDef_t CAN_Rx_TaControlBlock;
const osThreadAttr_t CAN_Rx_Ta_attributes = {
  .name = "CAN_Rx_Ta",
  .cb_mem = &CAN_Rx_TaControlBlock,
  .cb_size = sizeof(CAN_Rx_TaControlBlock),
  .stack_mem = &CAN_Rx_TaBuffer[0],
  .stack_size = sizeof(CAN_Rx_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DC_Motor_Ta */
osThreadId_t DC_Motor_TaHandle;
uint32_t DC_Motor_TaBuffer[ 256 ];
osStaticThreadDef_t DC_Motor_TaControlBlock;
const osThreadAttr_t DC_Motor_Ta_attributes = {
  .name = "DC_Motor_Ta",
  .cb_mem = &DC_Motor_TaControlBlock,
  .cb_size = sizeof(DC_Motor_TaControlBlock),
  .stack_mem = &DC_Motor_TaBuffer[0],
  .stack_size = sizeof(DC_Motor_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Servo_Ta */
osThreadId_t Servo_TaHandle;
uint32_t Servo_TaBuffer[ 256 ];
osStaticThreadDef_t Servo_TaControlBlock;
const osThreadAttr_t Servo_Ta_attributes = {
  .name = "Servo_Ta",
  .cb_mem = &Servo_TaControlBlock,
  .cb_size = sizeof(Servo_TaControlBlock),
  .stack_mem = &Servo_TaBuffer[0],
  .stack_size = sizeof(Servo_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Arm_State_Ta */
osThreadId_t Arm_State_TaHandle;
uint32_t Arm_State_TaBuffer[ 256 ];
osStaticThreadDef_t Arm_State_TaControlBlock;
const osThreadAttr_t Arm_State_Ta_attributes = {
  .name = "Arm_State_Ta",
  .cb_mem = &Arm_State_TaControlBlock,
  .cb_size = sizeof(Arm_State_TaControlBlock),
  .stack_mem = &Arm_State_TaBuffer[0],
  .stack_size = sizeof(Arm_State_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* Impl 函数声明 (实现在 App/tasks/) */
void Heartbeat_Task_Impl(void *argument);
void CAN_Rx_Task_Impl(void *argument);
void DC_Motor_Task_Impl(void *argument);
void Servo_Task_Impl(void *argument);
void Arm_State_Task_Impl(void *argument);
/* USER CODE END FunctionPrototypes */

void Heartbeat_Task_Run(void *argument);
void CAN_Rx_Task_Run(void *argument);
void DC_Motor_Task_Run(void *argument);
void Servo_Task_Run(void *argument);
void Arm_State_Task_Run(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (16, sizeof(App_CAN_Message_t), &canRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat_Ta */
  Heartbeat_TaHandle = osThreadNew(Heartbeat_Task_Run, NULL, &Heartbeat_Ta_attributes);

  /* creation of CAN_Rx_Ta */
  CAN_Rx_TaHandle = osThreadNew(CAN_Rx_Task_Run, NULL, &CAN_Rx_Ta_attributes);

  /* creation of DC_Motor_Ta */
  DC_Motor_TaHandle = osThreadNew(DC_Motor_Task_Run, NULL, &DC_Motor_Ta_attributes);

  /* creation of Servo_Ta */
  Servo_TaHandle = osThreadNew(Servo_Task_Run, NULL, &Servo_Ta_attributes);

  /* creation of Arm_State_Ta */
  Arm_State_TaHandle = osThreadNew(Arm_State_Task_Run, NULL, &Arm_State_Ta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Heartbeat_Task_Run */
/**
  * @brief  Function implementing the Heartbeat_Ta thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Heartbeat_Task_Run */
void Heartbeat_Task_Run(void *argument)
{
  /* USER CODE BEGIN Heartbeat_Task_Run */
  Heartbeat_Task_Impl(argument);
  /* USER CODE END Heartbeat_Task_Run */
}

/* USER CODE BEGIN Header_CAN_Rx_Task_Run */
/**
* @brief Function implementing the CAN_Rx_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Rx_Task_Run */
void CAN_Rx_Task_Run(void *argument)
{
  /* USER CODE BEGIN CAN_Rx_Task_Run */
  CAN_Rx_Task_Impl(argument);
  /* USER CODE END CAN_Rx_Task_Run */
}

/* USER CODE BEGIN Header_DC_Motor_Task_Run */
/**
* @brief Function implementing the DC_Motor_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DC_Motor_Task_Run */
void DC_Motor_Task_Run(void *argument)
{
  /* USER CODE BEGIN DC_Motor_Task_Run */
  DC_Motor_Task_Impl(argument);
  /* USER CODE END DC_Motor_Task_Run */
}

/* USER CODE BEGIN Header_Servo_Task_Run */
/**
* @brief Function implementing the Servo_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Task_Run */
void Servo_Task_Run(void *argument)
{
  /* USER CODE BEGIN Servo_Task_Run */
  Servo_Task_Impl(argument);
  /* USER CODE END Servo_Task_Run */
}

/* USER CODE BEGIN Header_Arm_State_Task_Run */
/**
* @brief Function implementing the Arm_State_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Arm_State_Task_Run */
void Arm_State_Task_Run(void *argument)
{
  /* USER CODE BEGIN Arm_State_Task_Run */
  Arm_State_Task_Impl(argument);
  /* USER CODE END Arm_State_Task_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

