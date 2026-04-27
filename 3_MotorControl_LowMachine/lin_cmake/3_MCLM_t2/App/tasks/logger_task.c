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

  // 方案C：local static buf — DMA 搬运期间不会被下一帧 snprintf 覆盖
  static uint8_t local_tx_buf[64];

  for(;;)
  {
    // 阻塞等待任意一路电机的数据帧（队列空则挂起，不占 CPU）
    LogMotorData_t log_data;
    osMessageQueueGet(LogQueueHandle, &log_data, 0, osWaitForever);

    if (!g_logger_enabled)
      continue;

    // 浮点转整数+小数（F103 无法直接用 %f）
    int32_t target_speed_int = (int32_t)log_data.target_logic_speed;
    int32_t target_speed_dec = (int32_t)(fabsf(log_data.target_logic_speed - (float)target_speed_int) * 10.0f);

    int len = snprintf((char *)local_tx_buf, sizeof(local_tx_buf),
                   "%lu,%u,%d,%ld.%ld,%d\r\n",
                   (unsigned long)log_data.timestamp_ms,
                   (unsigned int)log_data.motor_id,
                   (int)log_data.current_ticks,
                   (long int)target_speed_int,
                   (long int)target_speed_dec,
                   (int)log_data.pwm_output);

    // 信号量保护：等待上一次 DMA 传输完成后再发新帧，彻底避免 tx_buf 覆盖
    osSemaphoreAcquire(uart1_dma_semHandle, osWaitForever);
    HAL_UART_Transmit_DMA(&huart1, local_tx_buf, len);

    // 如果串口出现 ORE 错误，清除标志位防止死锁
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
  }
  /* USER CODE END Start_SerialLog */
}