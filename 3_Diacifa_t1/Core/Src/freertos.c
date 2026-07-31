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

/* CAN 命令队列：中断 → AirCtrl 任务的通信通道 */
osMessageQueueId_t CAN_CmdQueueHandle;
const osMessageQueueAttr_t CAN_CmdQueue_attributes = {
  .name = "CAN_CmdQueue"
};

/* 全局气动系统当前命令状态（AirCtrl_Ta 负责更新，Heartbeat_Ta 读取上报） */
AirCommand_t g_air_cmd = {0};

/* USER CODE END Variables */
/* Definitions for Heartbeat_Ta */
osThreadId_t Heartbeat_TaHandle;
uint32_t defaultTaskBuffer[ 64 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t Heartbeat_Ta_attributes = {
  .name = "Heartbeat_Ta",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_Rx_Ta */
osThreadId_t CAN_Rx_TaHandle;
uint32_t CAN_Rx_TaBuffer[ 128 ];
osStaticThreadDef_t CAN_Rx_TaControlBlock;
const osThreadAttr_t CAN_Rx_Ta_attributes = {
  .name = "CAN_Rx_Ta",
  .cb_mem = &CAN_Rx_TaControlBlock,
  .cb_size = sizeof(CAN_Rx_TaControlBlock),
  .stack_mem = &CAN_Rx_TaBuffer[0],
  .stack_size = sizeof(CAN_Rx_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for AirCtrl_Ta */
osThreadId_t AirCtrl_TaHandle;
uint32_t AirCtrl_TaBuffer[ 128 ];
osStaticThreadDef_t AirCtrl_TaControlBlock;
const osThreadAttr_t AirCtrl_Ta_attributes = {
  .name = "AirCtrl_Ta",
  .cb_mem = &AirCtrl_TaControlBlock,
  .cb_size = sizeof(AirCtrl_TaControlBlock),
  .stack_mem = &AirCtrl_TaBuffer[0],
  .stack_size = sizeof(AirCtrl_TaBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Heartbeat(void *argument);
void Start_CAN_Rx(void *argument);
void Start_AirControl(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* 创建 CAN 命令队列（中断 → AirCtrl 任务） */
  CAN_CmdQueueHandle = osMessageQueueNew(CAN_CMD_QUEUE_LENGTH, sizeof(AirCommand_t), &CAN_CmdQueue_attributes);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat_Ta */
  Heartbeat_TaHandle = osThreadNew(Start_Heartbeat, NULL, &Heartbeat_Ta_attributes);

  /* creation of CAN_Rx_Ta */
  CAN_Rx_TaHandle = osThreadNew(Start_CAN_Rx, NULL, &CAN_Rx_Ta_attributes);

  /* creation of AirCtrl_Ta */
  AirCtrl_TaHandle = osThreadNew(Start_AirControl, NULL, &AirCtrl_Ta_attributes);

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
  uint32_t last_report_tick = 0;

  for(;;)
  {
    /* LED 心跳：200ms 翻转一次 */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* CAN 状态上报：500ms 发送一次 */
    uint32_t now = HAL_GetTick();
    if (now - last_report_tick >= CAN_REPORT_PERIOD_MS) {
        last_report_tick = now;
        CAN_SendStatus(&hcan, &g_air_cmd, 0);
    }

    osDelay(HEARTBEAT_PERIOD_MS);
  }
  /* USER CODE END Start_Heartbeat */
}

/* USER CODE BEGIN Header_Start_CAN_Rx */
/**
* @brief Function implementing the CAN_Rx_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CAN_Rx */
void Start_CAN_Rx(void *argument)
{
  /* USER CODE BEGIN Start_CAN_Rx */
  /*
   * 当前设计：
   *   CAN RX 中断回调 → 解析 AirCommand → osMessageQueuePut(CAN_CmdQueue)
   *   AirCtrl_Ta 从 CAN_CmdQueue 读取并执行
   *
   * 本任务保留作为扩展点。启用 ABOM(自动离线恢复)后，
   * CAN 总线故障由硬件自动处理，无需软件干预。
   */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END Start_CAN_Rx */
}

/* USER CODE BEGIN Header_Start_AirControl */
/**
* @brief Function implementing the AirCtrl_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AirControl */
void Start_AirControl(void *argument)
{
  /* USER CODE BEGIN Start_AirControl */
  AirCommand_t cmd;

  /* 确保 TIM2 4 路 PWM 全部启动（每块 IBT4 并联 2 通道驱动一个泵） */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* 初始状态：全部关闭 */
  IBT4_Pump_SetSpeed(PUMP1_ID, 0);
  IBT4_Pump_SetSpeed(PUMP2_ID, 0);
  IBT4_Valve_Set(VALVE1_ID, 0);
  IBT4_Valve_Set(VALVE2_ID, 0);
  g_air_cmd.pump1_pwm = 0;
  g_air_cmd.pump2_pwm = 0;
  g_air_cmd.valve1_on = 0;
  g_air_cmd.valve2_on = 0;

  for(;;)
  {
    /* 阻塞等待 CAN 命令
     * 收到一条命令就设一次状态，之后一直保持直到下一条命令覆盖 */
    osMessageQueueGet(CAN_CmdQueueHandle, &cmd, NULL, osWaitForever);

    /* 执行命令 */
    IBT4_Pump_SetSpeed(PUMP1_ID, cmd.pump1_pwm);
    IBT4_Pump_SetSpeed(PUMP2_ID, cmd.pump2_pwm);
    IBT4_Valve_Set(VALVE1_ID, cmd.valve1_on);
    IBT4_Valve_Set(VALVE2_ID, cmd.valve2_on);

    /* 更新全局状态（给 Heartbeat 读去做 CAN 上报） */
    g_air_cmd = cmd;
  }
  /* USER CODE END Start_AirControl */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

