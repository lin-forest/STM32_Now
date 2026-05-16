#include "app_globals.h"
#include "app_includes.h"
#include <stdio.h>
#include <math.h>

void Logger_Task(void *argument)
{
  /* USER CODE BEGIN Start_SerialLog */
  /* Infinite loop */

  // main.c迁移过来的，对main.c进行解耦合
  Logger_Init();

  static uint8_t local_tx_buf[128];

  for(;;)
  {
    LogMotorData_t d0, d1;
    int merged = 0;

    // 阻塞取第一帧
    osMessageQueueGet(LogQueueHandle, &d0, 0, osWaitForever);

    if (!g_logger_enabled)
      continue;

    // 短超时取第二帧（配对合并为一帧，方便上位机单行解析两路）
    if (osMessageQueueGet(LogQueueHandle, &d1, 0, 20) == osOK) {
      merged = 1;
      if (d0.motor_id == 1) { LogMotorData_t t = d0; d0 = d1; d1 = t; }
    }

    osSemaphoreAcquire(uart1_dma_semHandle, osWaitForever);

    int len;
    if (merged) {
      int32_t s0i = (int32_t)d0.target_logic_speed;
      int32_t s0d = (int32_t)(fabsf(d0.target_logic_speed - (float)s0i) * 10.0f);
      int32_t s1i = (int32_t)d1.target_logic_speed;
      int32_t s1d = (int32_t)(fabsf(d1.target_logic_speed - (float)s1i) * 10.0f);

      len = snprintf((char *)local_tx_buf, sizeof(local_tx_buf),
                 "%lu,%d,%d,%ld.%ld,%d,%d,%d,%ld.%ld,%d\r\n",
                 (unsigned long)d0.timestamp_ms,
                 (int)d0.current_ticks, (int)d0.accumulated_ticks,
                 (long)s0i, (long)s0d, (int)d0.pwm_output,
                 (int)d1.current_ticks, (int)d1.accumulated_ticks,
                 (long)s1i, (long)s1d, (int)d1.pwm_output);
    } else {
      int32_t si = (int32_t)d0.target_logic_speed;
      int32_t sd = (int32_t)(fabsf(d0.target_logic_speed - (float)si) * 10.0f);

      len = snprintf((char *)local_tx_buf, sizeof(local_tx_buf),
                 "%lu,%u,%d,%ld.%ld,%d\r\n",
                 (unsigned long)d0.timestamp_ms,
                 (unsigned int)d0.motor_id,
                 (int)d0.current_ticks, (long)si, (long)sd, (int)d0.pwm_output);
    }

    HAL_UART_Transmit_DMA(&huart1, local_tx_buf, len);

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
      __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  /* USER CODE END Start_SerialLog */
}