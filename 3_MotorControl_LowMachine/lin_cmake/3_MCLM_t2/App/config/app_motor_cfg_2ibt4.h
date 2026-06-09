#ifndef __APP_MOTOR_CFG_2IBT4_H__
#define __APP_MOTOR_CFG_2IBT4_H__

/* =================================================================================
 *   [双 IBT-4 电机预设]
 *   MOTOR_CFG_SET 设为 MOTOR_CFG_2IBT4 后生效
 *
 *   电机1（转向）→ IBT-4 (BTS7960)
 *   电机2（动力）→ IBT-4 (BTS7960)
 * ================================================================================= */

#define MOTOR1_DRIVER   MOTOR_DRIVER_IBT4   // 转向电机
#define MOTOR2_DRIVER   MOTOR_DRIVER_IBT4   // 动力电机

/* SPEED_TICKS_MAX 计算:
 *   12 PPR × 4(TI12) × (8986 RPM / 60) × 0.01s ≈ 72  (理论值)
 *   增加 33% 裕量避免异常饱和 → 96
 *   实测输出轴 877 ticks/rev → 电机轴 48 ticks/rev, gear_ratio ≈ 18.27
 */
#define SPEED_TICKS_MAX     96    // 单控制周期编码器最大计数值 (用于速度计算)


/* =================================================================================
 *   2. 电机1（转向电机）— IBT-4 (BTS7960)
 *   通道分配: TIM1_CH3 = RPWM (PA10), TIM1_CH4 = LPWM (PA11)
 * ================================================================================= */

#if (MOTOR1_DRIVER == MOTOR_DRIVER_IBT4)

/* --- IBT-4 硬件配置 --- */
#define MOTOR1_IBT4_TIM             &htim1
#define MOTOR1_IBT4_CH_F            TIM_CHANNEL_3   // RPWM (正转) PA10
#define MOTOR1_IBT4_CH_R            TIM_CHANNEL_4   // LPWM (反转) PA11
#define MOTOR1_IBT4_EN_PORT         NULL
#define MOTOR1_IBT4_EN_PIN          0
#define MOTOR1_IBT4_POLARITY        0

/* --- 电机控制限制 --- */
#define MOTOR1_MAX_PWM_OUTPUT       PWM_MAX
#define MOTOR1_MIN_PWM_OUTPUT       0
#define MOTOR1_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
#define MOTOR1_DEAD_ZONE            10

/* --- PID控制器参数 --- */
#define MOTOR1_PID_KP                0.4584f
#define MOTOR1_PID_KI                17.66f
#define MOTOR1_PID_KD                0.0025f
#define MOTOR1_PID_INTEGRAL_LIMIT    5.66f
#define MOTOR1_PID_OUTPUT_LIMIT      100.0f
#define MOTOR1_PID_TS                0.01f
#define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f

/* --- 编码器 --- */
#define MOTOR1_ENCODER_TIM          &htim2          // TIM2 编码器模式 (PA0/PA1)

#else
  #error "MOTOR1_DRIVER must be MOTOR_DRIVER_IBT4 in this preset"
#endif


/* =================================================================================
 *   3. 电机2（动力电机）— IBT-4 (BTS7960)
 *   通道分配: TIM1_CH1 = RPWM (PA8), TIM1_CH2 = LPWM (PA9)
 * ================================================================================= */

#if (MOTOR2_DRIVER == MOTOR_DRIVER_IBT4)

/* --- IBT-4 硬件配置 --- */
#define MOTOR2_IBT4_TIM             &htim1
#define MOTOR2_IBT4_CH_F            TIM_CHANNEL_1   // RPWM (正转) PA8
#define MOTOR2_IBT4_CH_R            TIM_CHANNEL_2   // LPWM (反转) PA9
#define MOTOR2_IBT4_EN_PORT         NULL
#define MOTOR2_IBT4_EN_PIN          0
#define MOTOR2_IBT4_POLARITY        0

/* --- 电机控制限制 --- */
#define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
#define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
#define MOTOR2_DEAD_ZONE            10

/* --- PID控制器参数 --- */
#define MOTOR2_PID_KP               0.4584f
#define MOTOR2_PID_KI               17.66f
#define MOTOR2_PID_KD               0.0025f
#define MOTOR2_PID_INTEGRAL_LIMIT   5.66f
#define MOTOR2_PID_OUTPUT_LIMIT     100.0f
#define MOTOR2_PID_TS               0.01f
#define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f
      
/* --- 编码器 --- */
#define MOTOR2_ENCODER_TIM          &htim3          // TIM3 编码器模式 (PA6/PA7)

#else
  #error "MOTOR2_DRIVER must be MOTOR_DRIVER_IBT4 in this preset"
#endif

#endif // __APP_MOTOR_CFG_2IBT4_H__
