#include "app_globals.h"
#include "app_includes.h"
#include "cmsis_os2.h"
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
    // 阻塞等待来自 Encoder_Task 的标志位 (0x01)
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    if (!g_logger_enabled)
      continue;

    /*
     * 校准模式：直接读取原始编码器计数，不做任何映射。
     * 字段说明：
     *   hw_cnt  — 定时器寄存器当前值 (uint16, 0~65535)，实时反映硬件计数器位置
     *   ticks   — 本周期增量 (int16, 有符号)，10ms 内的脉冲变化量
     *   enc_abs — 软件累积绝对计数 (int32，可跨溢出，从上电或复位后开始累加)
     */
    uint16_t hw_cnt[MOTOR_COUNT];
    int16_t  ticks[MOTOR_COUNT];
    int32_t  enc_abs[MOTOR_COUNT];
    int16_t  mspeed[MOTOR_COUNT];
    int16_t  logic_speed[MOTOR_COUNT];


    /* 先在 Mutex 外读取硬件寄存器（原子读，不影响其他任务） */
    hw_cnt[0] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    hw_cnt[1] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    // --- Lock Mutex ---
    if (osMutexAcquire(motor_mutexHandle, 10) == osOK) // Wait max 10ms
    {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            ticks[i]   = g_motors[i].current_ticks;
            enc_abs[i] = g_motors[i].encoder_count;
            logic_speed[i] = g_motors[i].target_logic_speed;
            mspeed[i]  = g_motors[i].measured_speed;
        }

        // --- Release Mutex ---
        osMutexRelease(motor_mutexHandle);
    }
    else
    {
        // Failed to acquire mutex, skip this log cycle
        continue;
    }

    /*
     * 输出格式 (CSV，便于 Serial Studio / Python / 串口助手解析):
     *
     *   SysMs, M1_HW, M1_Target, M1_dTick, M1_Abs, M1_Speed, M2_HW, M2_Target, M2_dTick, M2_Abs, M2_Speed
     *   │       │       │          │         │        │         │       │          │          │       └─ M2 测量速度
     *   │       │       │          │         │        │         │       │          │          └───────── M2 累积绝对计数
     *   │       │       │          │         │        │         │       │          └──────────────────── M2 本周期增量
     *   │       │       │          │         │        │         │       └─────────────────────────────── M2 目标逻辑速度
     *   │       │       │          │         │        │         └───────────────────────────────────────  M2 TIM3 寄存器值
     *   │       │       │          │         │        └─────────────────────────────────────────────── M1 测量速度
     *   │       │       │          │         └──────────────────────────────────────────────────────── M1 累积绝对计数
     *   │       │       │          └────────────────────────────────────────────────────────────────── M1 本周期增量
     *   │       │       └───────────────────────────────────────────────────────────────────────────── M1 目标逻辑速度
     *   │       └───────────────────────────────────────────────────────────────────────────────────── M1 TIM2 寄存器值
     *   └───────────────────────────────────────────────────────────────────────────────────────────── FreeRTOS 系统时间 (ms)
     */
    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                   "%lu,%u,%d,%d,%ld,%d,%u,%d,%d,%ld,%d\r\n",
                   (unsigned long)osKernelGetTickCount(),
                   (unsigned)hw_cnt[0],
                   (int)logic_speed[0],
                   (int)ticks[0],
                   (long)enc_abs[0],
                   (int)mspeed[0],
                   (unsigned)hw_cnt[1],
                   (int)logic_speed[1],
                   (int)ticks[1],
                   (long)enc_abs[1],
                   (int)mspeed[1]);

    // 串口发送，DMA 方式
    // 只检查发送状态 (gState)，避免因为 RX 引脚浮空报错导致无法发送
    if (huart1.gState == HAL_UART_STATE_READY)
    {
      HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
    }

    // 如果串口出现错误（如溢出），清除标志位防止死锁
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
  }
  /* USER CODE END Start_SerialLog */
}
