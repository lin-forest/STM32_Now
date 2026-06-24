/* ============================================================
 *  servo.h — 舵机 PWM 控制
 *  35kg 数字舵机, 300°, 500μs~2500μs
 *  TIM4: PSC=17, ARR=39999 → 每 tick=0.5μs
 * ============================================================ */
#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"

void     Servo_Init(void);
uint16_t Servo_AngleToPulse(float angle_deg);
void     Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse);
void     Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg);

#endif
