#include "motor_DC_IBT4.h"
#include <stdint.h>

/* IBT-4 死区最小值，低于此值的 PWM 被忽略 */
#define IBT4_DEAD_ZONE_MIN  1

void IBT4_Motor_Init(IBT4_Motor_t *motor,
                     TIM_HandleTypeDef *htim,
                     uint32_t Channel_Forward,
                     uint32_t Channel_Reverse,
                     GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
                     uint16_t MaxPWM, int16_t MaxSpeed,
                     uint16_t DeadZone, uint8_t Polarity)
{
    motor->htim            = htim;
    motor->Channel_Forward = Channel_Forward;
    motor->Channel_Reverse = Channel_Reverse;
    motor->EN_Port         = EN_Port;
    motor->EN_Pin          = EN_Pin;
    motor->MaxPWM          = MaxPWM;
    motor->MaxSpeed        = MaxSpeed;
    motor->DeadZone        = (DeadZone < IBT4_DEAD_ZONE_MIN) ? IBT4_DEAD_ZONE_MIN : DeadZone;
    motor->Polarity        = (Polarity > 0) ? 1 : 0;
    motor->pwm_output      = 0;

    /* 启动两路 PWM (初始占空比 0) */
    if (htim != NULL) {
        HAL_TIM_PWM_Start(htim, Channel_Forward);
        HAL_TIM_PWM_Start(htim, Channel_Reverse);
        __HAL_TIM_SET_COMPARE(htim, Channel_Forward, 0);
        __HAL_TIM_SET_COMPARE(htim, Channel_Reverse, 0);
    }

    /* 使能引脚 (若有) */
    if (EN_Port != NULL && EN_Pin != 0) {
        HAL_GPIO_WritePin(EN_Port, EN_Pin, GPIO_PIN_SET);
    }
}

void IBT4_Motor_SetSpeed(IBT4_Motor_t *motor, int16_t speed)
{
    /* 限幅 */
    if (speed > motor->MaxSpeed)  speed = motor->MaxSpeed;
    if (speed < -motor->MaxSpeed) speed = -motor->MaxSpeed;

    /* 极性反转 */
    if (motor->Polarity) speed = -speed;

    /* 死区处理 */
    if (speed > 0 && speed < (int16_t)motor->DeadZone)  speed = motor->DeadZone;
    if (speed < 0 && speed > -(int16_t)motor->DeadZone) speed = -(int16_t)motor->DeadZone;

    /* 转换为 PWM 占空比 */
    uint16_t pwmVal = 0;
    if (motor->MaxSpeed != 0)
        pwmVal = (uint16_t)(((int32_t)(speed > 0 ? speed : -speed) * motor->MaxPWM) / motor->MaxSpeed);

    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Forward, pwmVal);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Reverse, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Forward, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Reverse, pwmVal);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Forward, 0);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Reverse, 0);
    }

    motor->pwm_output = pwmVal;
}

void IBT4_Motor_Stop(IBT4_Motor_t *motor)
{
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Forward, 0);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel_Reverse, 0);
    motor->pwm_output = 0;
}
