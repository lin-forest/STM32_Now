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
#include "app_globals.h"
#include "mt6701.h"
#include "servo.h"
#include "tim.h"
#include "gpio.h"
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
uint32_t Servo_TaBuffer[ 128 ];
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
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(300);
  }
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
  for(;;)
  {
    App_CAN_Message_t msg;
    if (osMessageQueueGet(canRxQueueHandle, &msg, NULL, osWaitForever) == osOK) {
        printf("CAN RX: ID=0x%03lX Len=%d ", msg.id, msg.len);
        for (int i = 0; i < msg.len; i++)
            printf("%02X ", msg.data[i]);
        printf("\r\n");
    }
  }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
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
  Servo_Init();
  MT6701_Init();

  /* 从中位向两侧扩展：找真实角度范围 */
  /* 角度约定：-150° ~ +150°，0° 为中位 */
  float angle = 0.0f;
  int16_t ccr;

  osDelay(2000);

  /* Phase 1：走到中位 0° */
  printf("=== CENTER (0°) ===\r\n");
  Servo_SetAngle(&htim4, TIM_CHANNEL_1, 0.0f);
  ccr = Servo_AngleToPulse(0.0f);
  printf("angle=0°  pulse=%uus  CCR=%u\r\n", ccr / 2, ccr);
  osDelay(2000);

  /* Phase 2：从 0° 逐步走到 +150° */
  printf("=== TO +150° ===\r\n");
  for (angle = 0.0f; angle <= 150.0f; angle += 10.0f) {
      Servo_SetAngle(&htim4, TIM_CHANNEL_1, angle);
      ccr = Servo_AngleToPulse(angle);
      printf("angle=%+4.0f°  pulse=%4uus  CCR=%u\r\n", angle, ccr / 2, ccr);
      osDelay(400);
  }

  /* Phase 3：从 +150° 逐步走到 -150° */
  printf("=== TO -150° ===\r\n");
  for (angle = 150.0f; angle >= -150.0f; angle -= 10.0f) {
      Servo_SetAngle(&htim4, TIM_CHANNEL_1, angle);
      ccr = Servo_AngleToPulse(angle);
      printf("angle=%+4.0f°  pulse=%4uus  CCR=%u\r\n", angle, ccr / 2, ccr);
      osDelay(400);
  }

  /* Phase 4：回到中位 0° */
  printf("=== BACK TO 0° ===\r\n");
  Servo_SetAngle(&htim4, TIM_CHANNEL_1, 0.0f);
  ccr = Servo_AngleToPulse(0.0f);
  printf("angle=0°  pulse=%uus  CCR=%u\r\n", ccr / 2, ccr);
  osDelay(2000);

  /* Phase 5：循环 */
  printf("=== REPEAT ===\r\n");
  for(;;) { osDelay(1000); }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Arm_State_Task_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

