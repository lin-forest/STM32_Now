#ifndef __APP_MOTOR_CFG_DEFAULT_H__
#define __APP_MOTOR_CFG_DEFAULT_H__

/* =================================================================================
 *   [默认预设] 原底盘配置
 *   电机1（转向）→ TB6612
 *   电机2（动力）→ TB6612
 * ================================================================================= */

#define MOTOR1_DRIVER   MOTOR_DRIVER_TB6612
#define MOTOR2_DRIVER   MOTOR_DRIVER_TB6612

#define SPEED_TICKS_MAX     90    // 单个控制周期内编码器的最大计数值 (用于速度计算)

/* =================================================================================
 *   电机1（转向电机）— TB6612
 * ================================================================================= */

/* --- TB6612 硬件配置 --- */
#define MOTOR1_TIM_HANDLE           &htim1
#define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1 // PWM信号引脚
#define MOTOR1_IN1_PORT             GPIOB         // 方向控制引脚1
#define MOTOR1_IN1_PIN              GPIO_PIN_0
#define MOTOR1_IN2_PORT             GPIOB         // 方向控制引脚2
#define MOTOR1_IN2_PIN              GPIO_PIN_1
#define MOTOR1_EN_PORT              NULL         // 使能引脚
#define MOTOR1_EN_PIN               0
#define MOTOR1_DEFAULT_STOP_MODE    TB6612_MOTOR_STOP_BRAKE // 默认停止模式

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
#define MOTOR1_ENCODER_TIM          &htim2


/* =================================================================================
 *   电机2（动力电机）— TB6612
 * ================================================================================= */

#define MOTOR2_TIM_HANDLE           &htim1
#define MOTOR2_TIM_CHANNEL          TIM_CHANNEL_2
#define MOTOR2_IN1_PORT             GPIOB
#define MOTOR2_IN1_PIN              GPIO_PIN_12
#define MOTOR2_IN2_PORT             GPIOB
#define MOTOR2_IN2_PIN              GPIO_PIN_13
#define MOTOR2_EN_PORT              NULL
#define MOTOR2_EN_PIN               0

#define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
#define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
#define MOTOR2_DEAD_ZONE            10

#define MOTOR2_PID_KP               0.4584f
#define MOTOR2_PID_KI               17.66f
#define MOTOR2_PID_KD               0.0025f
#define MOTOR2_PID_INTEGRAL_LIMIT   5.66f
#define MOTOR2_PID_OUTPUT_LIMIT     100.0f
#define MOTOR2_PID_TS               0.01f
#define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f

#define MOTOR1_ENCODER_TIM          &htim2
#define MOTOR2_ENCODER_TIM          &htim3

#endif // __APP_MOTOR_CFG_DEFAULT_H__
