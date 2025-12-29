#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* ===================== Speed Normalization ===================== */
/* 编码器在一个控制周期内的最大可达 tick */
#define SPEED_TICKS_MAX     80

/* 逻辑速度范围（对外接口统一） */
#define SPEED_LOGIC_MAX    100

/* PWM 输出上限（与 TIM 配置一致） */
#define PWM_MAX            999

// PID Constants for Motor Control
// #define MOTOR_PID_KP 2.0f
// #define MOTOR_PID_KI 1.0f
// #define MOTOR_PID_KD 0.5f

#endif // APP_CONFIG_H