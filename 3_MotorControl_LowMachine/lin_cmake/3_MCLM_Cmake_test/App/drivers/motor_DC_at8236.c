// #include "motor_DC_at8236.h"
// #include <stdlib.h>

// void At8236_Motor_Init(At8236_Motor_t *Motor, TIM_HandleTypeDef *PwmTimer, uint32_t Channel1, uint32_t Channel2, uint16_t MaxPwm, int16_t MaxSpeed)
// {
//     Motor->PwmTimer = PwmTimer;
//     Motor->Channel1 = Channel1;
//     Motor->Channel2 = Channel2;
//     Motor->MaxPwm = MaxPwm;
//     Motor->MinPwm = 0; // Default to 0, can be set later
//     Motor->MaxSpeed = MaxSpeed;

//     HAL_TIM_PWM_Start(Motor->PwmTimer, Motor->Channel1);
//     HAL_TIM_PWM_Start(Motor->PwmTimer, Motor->Channel2);
// }

// void At8236_Motor_SetSpeed(At8236_Motor_t *Motor, int16_t Speed)
// {
//     if (Speed > Motor->MaxSpeed)
//     {
//         Speed = Motor->MaxSpeed;
//     }
//     else if (Speed < -Motor->MaxSpeed)
//     {
//         Speed = -Motor->MaxSpeed;
//     }

//     uint32_t PwmValue = 0;
//     if (Speed != 0)
//     {
//         // Remap speed from [0, MaxSpeed] to [MinPwm, MaxPwm]
//         PwmValue = (uint32_t)abs(Speed) * (Motor->MaxPwm - Motor->MinPwm) / Motor->MaxSpeed + Motor->MinPwm;
//         if (PwmValue > Motor->MaxPwm)
//         {
//             PwmValue = Motor->MaxPwm;
//         }
//     }

//     if (Speed > 0)
//     {
//         // Forward
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, PwmValue);
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);
//     }
//     else if (Speed < 0)
//     {
//         // Reverse
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, PwmValue);
//     }
//     else
//     {
//         // Stop (Coast)
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);
//     }
// }

// void At8236_Motor_Stop(At8236_Motor_t *Motor, At8236_MotorStopMode_t StopMode)
// {
//     if (StopMode == AT8236_STOP_COAST)
//     {
//         // Coast
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, 0);
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, 0);
//     }
//     else
//     {
//         // Brake
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel1, Motor->MaxPwm);
//         __HAL_TIM_SET_COMPARE(Motor->PwmTimer, Motor->Channel2, Motor->MaxPwm);
//     }
// }