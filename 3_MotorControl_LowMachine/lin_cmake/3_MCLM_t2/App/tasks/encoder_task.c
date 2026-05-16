#include "app_globals.h"
#include "app_includes.h"
#include "speed_map.h"
#include "filter.h"


void Encoder_Task(void *argument)
{
    Motor_t *motor = (argument != NULL) ? (Motor_t *)argument : &g_motors[0];
    uint8_t  idx   = (uint8_t)(motor - &g_motors[0]);

    // 按 idx 选编码器定时器
    TIM_HandleTypeDef *htim_enc = (idx == 0) ? MOTOR1_ENCODER_TIM : MOTOR2_ENCODER_TIM;
    osMutexId_t        myMutex  = (idx == 0) ? motor0_mutexHandle  : motor1_mutexHandle;

    HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);

    int16_t last_cnt = 0;

    for(;;)
    {
        int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc);
        int16_t diff = (int16_t)(now - last_cnt);
        last_cnt = now;

        if (osMutexAcquire(myMutex, osWaitForever) == osOK)
        {
            motor->current_ticks       = diff;
            motor->accumulated_ticks  += diff;   // 累计编码器位置
            motor->current_logic_speed = ticks_to_logic(diff);
            osMutexRelease(myMutex);

            // 方案C：两路电机各自封包投递到 LogQueue，Logger 统一消费
            LogMotorData_t log_data = {
                .motor_id           = idx,
                .current_ticks      = diff,
                .accumulated_ticks  = motor->accumulated_ticks,
                .target_logic_speed = motor->target_logic_speed,
                .pwm_output         = motor->pwm_output,
                .timestamp_ms       = HAL_GetTick(),
            };
            osMessageQueuePut(LogQueueHandle, &log_data, 0, 0);
        }

        osDelay(10);
    }
}