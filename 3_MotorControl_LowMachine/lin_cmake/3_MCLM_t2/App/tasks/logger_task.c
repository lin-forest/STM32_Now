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

  for(;;)
  {
    // 阻塞等待来自 TIM3 中断的标志位 (0x01)
    // osThreadFlagsWait(flags, options, timeout)
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    if (!g_logger_enabled)
      continue;

    int32_t speed_val;
    float   target_logic_speed;
    int16_t pwm_output;
    uint32_t cnt_val = __HAL_TIM_GET_COUNTER(&htim2); 

    // --- Lock Mutex ---
    if (osMutexAcquire(motor_mutexHandle, 10) == osOK) // Wait max 10ms
    {
        speed_val = g_motors[0].current_ticks;
        target_logic_speed = g_motors[0].target_logic_speed;
        pwm_output = g_motors[0].pwm_output;
        
        // --- Release Mutex ---        
        osMutexRelease(motor_mutexHandle);
    }
    else
    {
        // Failed to acquire mutex, maybe skip this log cycle
        continue;
    }
    
    // int len = snprintf((char *)tx_buf, sizeof(tx_buf),
    //                "%lu,%lu,%d,%.1f,%d\r\n",
    //                (unsigned long)HAL_GetTick(),
    //                (unsigned long)cnt_val,
    //                (int)speed_val,
    //                target_logic_speed,
    //                (int)pwm_output);

    // 无法输出浮点数，当前是f103c8t6，如果需要输出浮点数，可以考虑以下两种方案：
    // 1. 使用 sprintf 替代 snprintf，并且在编译选项中添加 -u _printf_float 来支持浮点数输出
    // 2. 手动将浮点数转换为字符串，例如通过乘以10或100来保留一位或两位小数，然后输出整数部分和小数部分
    // 例如：
    int32_t target_speed_int = (int32_t)target_logic_speed;
    // 处理负数小数部分的显示
    int32_t target_speed_dec = (int32_t)(fabsf(target_logic_speed - (float)target_speed_int) * 10.0f);

    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                   "%lu,%lu,%d,%ld.%ld,%d\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned long)cnt_val,
                   (int)speed_val,
                   (long int)target_speed_int,
                   (long int)target_speed_dec,
                   (int)pwm_output);

    // 3. 串口发送，DMA 方式
    // 修改点：只检查发送状态 (gState)，避免因为 RX 引脚浮空报错导致无法发送
    if (huart1.gState == HAL_UART_STATE_READY)
    {
      HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
    }

    // 如果串口出现错误（如溢出），在此处尝试清除标志位，防止死锁
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
  }
  /* USER CODE END Start_SerialLog */
}