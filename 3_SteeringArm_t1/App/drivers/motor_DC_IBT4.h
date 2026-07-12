#ifndef __MOTOR_DC_IBT4_H
#define __MOTOR_DC_IBT4_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IBT-4 (BTS7960) 电机结构体
 *        RPWM = Channel_Forward,  LPWM = Channel_Reverse
 *        占空比 > 0 → RPWM 输出, LPWM = 0 (正转)
 *        占空比 < 0 → RPWM = 0, LPWM 输出 (反转)
 */
typedef struct {
    TIM_HandleTypeDef *htim;           // PWM 定时器
    uint32_t Channel_Forward;          // 正转 PWM 通道 (RPWM)
    uint32_t Channel_Reverse;          // 反转 PWM 通道 (LPWM)
    GPIO_TypeDef *EN_Port;             // 使能端口 (可选, 无则 NULL)
    uint16_t EN_Pin;                   // 使能引脚

    uint16_t MaxPWM;                   // 最大 PWM 值 (TIM ARR)
    int16_t  MaxSpeed;                 // 最大逻辑速度
    uint16_t DeadZone;                 // 死区
    uint8_t  Polarity;                 // 极性 (0=默认, 1=反转)

    int16_t  pwm_output;               // 当前 PWM 输出值
} IBT4_Motor_t;

void IBT4_Motor_Init(IBT4_Motor_t *motor,
                     TIM_HandleTypeDef *htim,
                     uint32_t Channel_Forward,
                     uint32_t Channel_Reverse,
                     GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
                     uint16_t MaxPWM, int16_t MaxSpeed,
                     uint16_t DeadZone, uint8_t Polarity);
void IBT4_Motor_SetSpeed(IBT4_Motor_t *motor, int16_t speed);
void IBT4_Motor_Stop(IBT4_Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_DC_IBT4_H
