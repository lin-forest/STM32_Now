/* ============================================================
 *  servo.c — 舵机 PWM 控制
 *  参考 RC_dog 架构：-half_range ~ +half_range, 0° 为中位
 *
 *  TIM4: PSC=35, ARR=39999, 72MHz 定时器时钟
 *  每 tick = 0.5μs, CCR = pulse_us × 2
 *
 *  脉宽映射（phys_range=300, pulse_min=500, pulse_max=2500）:
 *    out_angle = angle + half_range     ← 从 ±150° 归一化到 0~300
 *    pulse_us  = 500 + out_angle × 2000/300
 *    CCR       = pulse_us × 2
 *
 *  验证:
 *    angle=-150° → out=0   → 500μs  → CCR=1000 ✅
 *    angle=0°    → out=150 → 1500μs → CCR=3000 ✅
 *    angle=+150° → out=300 → 2500μs → CCR=5000 ✅
 * ============================================================ */
#include "servo.h"
#include "tim.h"

#define PHYS_RANGE      300.0f   /* 舵机物理总范围 */
#define HALF_RANGE      150.0f   /* ±150° */
#define PULSE_MIN       500      /* 最小脉宽 μs */
#define PULSE_MAX       2500     /* 最大脉宽 μs */
#define TICK_US         0.5f     /* 每 tick = 0.5μs */

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* 初始：J1/J2 = 0° (中位), 夹爪 = 全开 */
    Servo_SetAngle(&htim4, TIM_CHANNEL_1, 0.0f);
    Servo_SetAngle(&htim4, TIM_CHANNEL_2, 0.0f);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);
}

/* 角度 → 脉宽 (μs)
 *   out_angle = angle + HALF_RANGE     // 归一化到 0~PHYS_RANGE
 *   pulse_us  = PULSE_MIN + out_angle × (PULSE_MAX-PULSE_MIN) / PHYS_RANGE
 */
uint16_t Servo_AngleToPulse(float angle_deg)
{
    if (angle_deg < -HALF_RANGE) angle_deg = -HALF_RANGE;
    if (angle_deg >  HALF_RANGE) angle_deg =  HALF_RANGE;

    float out_angle = angle_deg + HALF_RANGE;         /* [-150,+150] → [0,300] */
    float pulse_us = (float)PULSE_MIN + out_angle * (float)(PULSE_MAX - PULSE_MIN) / PHYS_RANGE;

    return (uint16_t)(pulse_us / TICK_US + 0.5f);     /* μs → CCR, 四舍五入 */
}

void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{
    if (pulse < (uint16_t)(PULSE_MIN / TICK_US)) pulse = (uint16_t)(PULSE_MIN / TICK_US);
    if (pulse > (uint16_t)(PULSE_MAX / TICK_US)) pulse = (uint16_t)(PULSE_MAX / TICK_US);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
{
    Servo_SetPulse(htim, channel, Servo_AngleToPulse(angle_deg));
}
