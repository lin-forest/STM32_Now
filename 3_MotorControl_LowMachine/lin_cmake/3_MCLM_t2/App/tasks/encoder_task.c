#include "app_globals.h"
#include "app_includes.h"
/* speed_map 和 filter 暂时不使用，以便直接观察原始编码器计数 */
/* #include "speed_map.h" */
/* #include "filter.h"   */


void Encoder_Task(void *argument)
{
  /* 启动电机 1 和电机 2 对应的编码器定时器 */
  HAL_TIM_Encoder_Start(MOTOR1_ENCODER_TIM, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(MOTOR2_ENCODER_TIM, TIM_CHANNEL_ALL);

  /*
   * 使用 uint16_t 保存上一次的原始计数值。
   * TIM2/TIM3 的计数寄存器是无符号 16 位 (0~65535)，用 int16_t 接收
   * 会导致符号位截断，造成差值在 0~1 之间错误跳变。
   */
  static uint16_t last_cnts[MOTOR_COUNT] = {0};
  TIM_HandleTypeDef* encoder_tims[MOTOR_COUNT] = {MOTOR1_ENCODER_TIM, MOTOR2_ENCODER_TIM};

  /* ── 滑动平均滤波器（N=4，平衡噪声抑制与响应速度）──────────────────────
   * 原始 diff 每周期最大变化 ≈ ±90 ticks，单点噪声可达 ±2~5 ticks。
   * N=4 → 40ms 均值窗口，可将噪声标准差降低 50%，同时保持足够动态响应。
   * ─────────────────────────────────────────────────────────────────────── */
  #define SPEED_FILTER_N  4
  static int16_t speed_buf[MOTOR_COUNT][SPEED_FILTER_N] = {{0}};
  static uint8_t speed_buf_idx[MOTOR_COUNT] = {0};

  for(;;)
  {
    if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
    {
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            /*
             * 读取 16 位原始计数值 (0~65535)。
             * 用 uint16_t 接收以保留完整无符号范围，
             * 差值转换为 int16_t 可自动处理正转/反转时的环绕溢出。
             */
            uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tims[i]);
            int16_t  diff = (int16_t)(now - last_cnts[i]);
            last_cnts[i]  = now;

            /*
             * current_ticks   : 本周期增量 (有符号，正转为正，反转为负)
             * encoder_count   : 绝对累积计数，用于校准编码器线数
             * measured_speed  : 暂时直接存原始增量，不做映射，
             *                   方便在上位机直接观察真实脉冲数
             */
            g_motors[i].current_ticks       = diff;
            g_motors[i].encoder_count       += diff;       /* 累积绝对计数 */

            /* ── 滑动平均：写入环形缓冲，取 N 点均值 ── */
            speed_buf[i][speed_buf_idx[i]] = diff;
            speed_buf_idx[i] = (speed_buf_idx[i] + 1) % SPEED_FILTER_N;
            int32_t sum = 0;
            for (int k = 0; k < SPEED_FILTER_N; k++) sum += speed_buf[i][k];
            g_motors[i].measured_speed = (float)sum / (float)SPEED_FILTER_N;
        }

        osMutexRelease(motor_mutexHandle);

        /* 数据更新成功后才唤醒 LogTask */
        if (Logger_TaHandle != NULL)
            osThreadFlagsSet(Logger_TaHandle, 0x01);
    }

    osDelay(10); // 10ms周期
  }
}