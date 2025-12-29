#include "app_includes.h"
#include "speed_map.h"


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

    // --- Lock Mutex ---
    if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
    {
        tb6612_motor1.current_ticks = diff;
        tb6612_motor1.current_logic_speed = ticks_to_logic(diff); // 将实际速度赋值给 current_logic_speed
        
        // --- Release Mutex ---
        osMutexRelease(motor_mutexHandle);
    }

    // 2. 唤醒 LogTask (使用 CMSIS V2 任务通知)
    if (Logger_TaHandle != NULL)
    {
      osThreadFlagsSet(Logger_TaHandle, 0x01); // 设置标志位 0x01 唤醒 LogTask
    }
    osDelay(10); // 10ms 测速周期可行；1ms不行，完不成就过掉了
  }
  /* USER CODE END Start_SpeedMeasure */
}