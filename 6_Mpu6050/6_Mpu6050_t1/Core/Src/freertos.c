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
#include "mpu6050.h"
#include "imu_process.h"
#include "stdio.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
/* Definitions for Heartbeat_TA */
osThreadId_t Heartbeat_TAHandle;
const osThreadAttr_t Heartbeat_TA_attributes = {
  .name = "Heartbeat_TA",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Imu_TA */
osThreadId_t Imu_TAHandle;
const osThreadAttr_t Imu_TA_attributes = {
  .name = "Imu_TA",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Heartbeat_TA(void *argument);
void Start_Imu_TA(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat_TA */
  Heartbeat_TAHandle = osThreadNew(Start_Heartbeat_TA, NULL, &Heartbeat_TA_attributes);

  /* creation of Imu_TA */
  Imu_TAHandle = osThreadNew(Start_Imu_TA, NULL, &Imu_TA_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Heartbeat_TA */
/**
  * @brief  Function implementing the Heartbeat_TA thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Heartbeat_TA */
void Start_Heartbeat_TA(void *argument)
{
  /* USER CODE BEGIN Start_Heartbeat_TA */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500);
  }
  /* USER CODE END Start_Heartbeat_TA */
}

/* USER CODE BEGIN Header_Start_Imu_TA */
/**
* @brief Function implementing the Imu_TA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Imu_TA */
void Start_Imu_TA(void *argument)
{
  /* USER CODE BEGIN Start_Imu_TA */
  IMU_Data_t imu_data = {0}; // 内部计算使用
  IMU_Output_t imu_output = {0}; // 标准化输出使用

  // 注意: IMU_Process_Init 现在需要 imu_data 的地址来初始化四元数
  IMU_Process_Init(&hi2c1, &imu_data);

  uint32_t last_tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    uint32_t current_tick = osKernelGetTickCount();
    float dt = (current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    IMU_Process_Update(&hi2c1, &imu_data, &imu_output, dt);

    // 打印四元数和调试用的欧拉角 (乘以100后转为整数，避免使用浮点printf)
    // printf("Quat(x100): w:%ld x:%ld y:%ld z:%ld | Att(deg*10): P:%ld R:%ld Y:%ld\r\n",
    //        (long)(imu_output.orientation[0] * 100),
    //        (long)(imu_output.orientation[1] * 100),
    //        (long)(imu_output.orientation[2] * 100),
    //        (long)(imu_output.orientation[3] * 100),
    //        (long)(imu_data.Pitch * 10),
    //        (long)(imu_data.Roll * 10),
    //        (long)(imu_data.Yaw * 10));

    // printf("%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
    //        (long)(imu_output.orientation[0] * 100),
    //        (long)(imu_output.orientation[1] * 100),
    //        (long)(imu_output.orientation[2] * 100),
    //        (long)(imu_output.orientation[3] * 100),
    //        (long)(imu_data.Pitch * 10),
    //        (long)(imu_data.Roll * 10),
    //        (long)(imu_data.Yaw * 10));

    printf("%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            (long)(imu_output.orientation[0]         * 100),   // w
            (long)(imu_output.orientation[1]         * 100),   // x
            (long)(imu_output.orientation[2]         * 100),   // y
            (long)(imu_output.orientation[3]         * 100),   // z
            (long)(imu_output.linear_acceleration[0] * 100),   // ax m/s²
            (long)(imu_output.linear_acceleration[1] * 100),   // ay
            (long)(imu_output.linear_acceleration[2] * 100),   // az
            (long)(imu_output.angular_velocity[0]    * 100),   // gx rad/s
            (long)(imu_output.angular_velocity[1]    * 100),   // gy
            (long)(imu_output.angular_velocity[2]    * 100),   // gz
            (long)(imu_data.Pitch * 10),                        // Pitch °
            (long)(imu_data.Roll  * 10),                        // Roll  °
            (long)(imu_data.Yaw   * 10),                        // Yaw   °
            (long)(imu_output.magnetic_field[0] * 10),          // mx μT
            (long)(imu_output.magnetic_field[1] * 10),          // my μT
            (long)(imu_output.magnetic_field[2] * 10));         // mz μT

    osDelay(10);
  }
  /* USER CODE END Start_Imu_TA */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

