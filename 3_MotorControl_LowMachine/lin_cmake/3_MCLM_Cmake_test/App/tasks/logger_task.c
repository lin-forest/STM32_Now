#include "app_includes.h"

void Logger_Task(void *argument)
{
  /* USER CODE BEGIN Start_SerialLog */
  /* Infinite loop */

  // main.c迁移过来的，对main.c进行解耦合
  Logger_Init();

  for(;;)
  {
    // 阻塞等待来自 TIM3 中断的标志位 (0x01)
    // osThreadFlagsWait(flags, options, timeout)
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    int16_t speed_val;
    int16_t target_logic_speed;
    int16_t pwm_output;
    uint32_t cnt_val = __HAL_TIM_GET_COUNTER(&htim2); 

    // --- Lock Mutex ---
    if (osMutexAcquire(motor_mutexHandle, 10) == osOK) // Wait max 10ms
    {
        speed_val = tb6612_motor1.current_ticks;
        target_logic_speed = tb6612_motor1.target_logic_speed;
        pwm_output = tb6612_motor1.pwm_output;
        
        // --- Release Mutex ---
        osMutexRelease(motor_mutexHandle);
    }
    else
    {
        // Failed to acquire mutex, maybe skip this log cycle
        continue;
    }

    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                   "%lu,%lu,%d,%d,%d\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)cnt_val,
                   (int)speed_val,
                   (int)target_logic_speed,
                   (int)pwm_output);


    // 3. 串口发送，DMA 方式
    // HAL_UART_Transmit(&huart1, tx_buf, len, HAL_MAX_DELAY);
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
    {
      HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
    }
    else 
    {
      // If UART is busy, we might just skip this log frame.
      // The osDelay(5) could be removed if we want to immediately
      // go back to waiting for the next flag.
    }
    
    // This delay is redundant because the task is synchronized by osThreadFlagsWait
    // osDelay(10); 
  }
  /* USER CODE END Start_SerialLog */
}