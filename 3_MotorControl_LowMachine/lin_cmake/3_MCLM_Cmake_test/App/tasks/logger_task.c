
#include "app_includes.h"

void Logger_Task(void *argument)
{
  /* USER CODE BEGIN Start_SerialLog */
  /* Infinite loop */

  // main.c迁移过来的，对main.c进行解耦合
  UART_App_Init();

  for(;;)
  {
    // 阻塞等待来自 TIM3 中断的标志位 (0x01)
    // osThreadFlagsWait(flags, options, timeout)
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    // 收到标志位 (每 10ms 唤醒一次) 后执行耗时 I/O：

    // 1. 安全地读取速度
    int16_t speed_val = motor1.current_ticks;
    uint32_t cnt_val = __HAL_TIM_GET_COUNTER(&htim2); 

    // // 2. 格式化数据
    // int len = sprintf((char *)tx_buf, "%ld,%ld,%ld,%ld,%ld\r\n",HAL_GetTick(), cnt_val, speed_val,
    //                  - motor1.target_speed, motor1.pwm_output);

    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                   "%lu,%lu,%d,%d,%d\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)cnt_val,
                   (int)speed_val,
                   (int)(motor1.target_logic_speed),
                   (int)motor1.pwm_output);


    // 3. 串口发送，DMA 方式
    // HAL_UART_Transmit(&huart1, tx_buf, len, HAL_MAX_DELAY);
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
    {
      HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
    }
    else 
    {
      osDelay(5); // 如果 UART 忙，稍等一下再发
    }
    // HAL_UART_Transmit_DMA(&huart1, tx_buf, len);

    osDelay(10);
  }
  /* USER CODE END Start_SerialLog */
}


// #include "cmsis_os.h"
// #include "app_task.h"
// #include "logger.h"
// #include "string.h"
// // #include "cmsis_os.h"
// #include "tb6612_DC.h"
// // #include "task.h"
// #include "command.h"
// #include "tim.h"   // ← 必须加
// #include "app_task.h"
