
#include "app_includes.h"

void Encoder_Task(void *argument)
{
  /* USER CODE BEGIN Start_SpeedMeasure */
  /* Infinite loop */

  // main.c迁移
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  for(;;)
  {
    static int16_t last_cnt = 0; 
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);


    int32_t diff = (int16_t)(now - last_cnt);
    if (diff > 32768) 
    {
      diff -= 65536;
    }
    else if (diff < -32768) 
    {
      diff += 65536;
    }
    last_cnt = now;
    motor1.current_speed = diff;


    // // 2025.10.27 改进速度计算，解决计数器溢出问题
    // int16_t diff = (int16_t)(now - last_cnt);
    // last_cnt = now;
    // current_speed = diff;  // 保留符号方向

    // 1. 速度计算 (快速操作)
    // current_speed =  (now - last_cnt);
    // current_speed = - (now - last_cnt);
    // last_cnt = now;

    // 2. 唤醒 LogTask (使用 CMSIS V2 任务通知)
    if (Logger_TaHandle != NULL)
    {
      osThreadFlagsSet(Logger_TaHandle, 0x01); // 设置标志位 0x01 唤醒 LogTask
    }
    osDelay(10); // 10ms 测速周期可行；1ms不行，完不成就过掉了
  }
  /* USER CODE END Start_SpeedMeasure */
}


// #include "cmsis_os.h"
// #include "tb6612_DC.h"
// // #include "task.h"
// #include "command.h"
// #include "tim.h"   // ← 必须加
// #include "app_task.h"
