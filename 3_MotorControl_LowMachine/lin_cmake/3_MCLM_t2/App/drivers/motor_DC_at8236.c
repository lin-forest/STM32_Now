#include "motor_DC_at8236.h"
#include <stdlib.h> // For abs() function

void At8236_Motor_Init(At8236_Motor_t *Motor, TIM_HandleTypeDef *PwmTimer, uint32_t Channel1, uint32_t Channel2, uint16_t MaxPwm, int16_t MaxSpeed)
{
    Motor->PwmTimer = PwmTimer;
    Motor->Channel1 = Channel1;
    Motor->Channel2 = Channel2;
    Motor->MaxPwm = MaxPwm;
    Motor->MinPwm = 0; // Default to 0, can be set later if needed for static friction
    Motor->MaxSpeed = MaxSpeed;

    // 启动PWM输出
    HAL_TIM_PWM_Start(Motor->PwmTimer, Motor->Channel1);
    HAL_TIM_PWM_Start(Motor->PwmTimer, Motor->Channel2);
}

void At8236_Motor_SetSpeed(At8236_Motor_t *Motor, int16_t Speed)
{
    Motor->pwm_output = Speed;
    // 限制速度在最大允许范围内
    if (Speed > Motor->MaxSpeed)
    {
        Speed = Motor->MaxSpeed;
    }
    else if (Speed < -Motor->MaxSpeed)
    {
        Speed = -Motor->MaxSpeed;
    }

    uint32_t PwmValue = 0;
    if (Speed != 0)
    {
        // 将速度从 [0, MaxSpeed] 映射到 [MinPwm, MaxPwm]
        // 这里的PwmValue将用于PWM模式下的占空比
        PwmValue = (uint32_t)abs(Speed) * (Motor->MaxPwm - Motor->MinPwm) / Motor->MaxSpeed + Motor->MinPwm;
        if (PwmValue > Motor->MaxPwm)
        {
            PwmValue = Motor->MaxPwm;
        }
    }

    if (Speed > 0)
    {
        // 正向转动 (IN1 = PWM, IN2 = 0) - 对应AT8236的快衰减模式
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, PwmValue); // IN1输出PWM
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);        // IN2输出0 (低电平)
    }
    else if (Speed < 0)
    {
        // 反向转动 (IN1 = 0, IN2 = PWM) - 对应AT8236的快衰减模式
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);        // IN1输出0 (低电平)
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, PwmValue); // IN2输出PWM
    }
    else
    {
        // 停止 (滑行/Coast) (IN1 = 0, IN2 = 0)
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);
    }
}

void At8236_Motor_Stop(At8236_Motor_t *Motor, At8236_MotorStopMode_t StopMode)
{
    if (StopMode == AT8236_STOP_COAST)
    {
        // 滑行 (IN1 = 0, IN2 = 0)
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);
    }
    else // AT8236_STOP_BRAKE
    {
        // 刹车 (IN1 = 1, IN2 = 1)
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, Motor->MaxPwm); // IN1输出1 (高电平)
        __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, Motor->MaxPwm); // IN2输出1 (高电平)
    }
}