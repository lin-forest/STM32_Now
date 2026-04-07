#include "app_includes.h"
#include "speed_map.h"
#include "filter.h"


void Encoder_Task(void *argument)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  int16_t last_cnt = 0;

  for(;;)
  {
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

    int16_t diff = (int16_t)(now - last_cnt);
    last_cnt = now;

    if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
    {
        g_motor_status.current_ticks = diff;
        g_motor_status.current_logic_speed = ticks_to_logic(diff);
        // g_motor_status.current_logic_speed = ticks_to_logic(iir_filter(diff, ENCODER_FILTER_ALPHA));
        osMutexRelease(motor_mutexHandle);

        // 数据更新成功后才唤醒 LogTask
        if (Logger_TaHandle != NULL)
            osThreadFlagsSet(Logger_TaHandle, 0x01);
    }

    osDelay(10);
  }
}