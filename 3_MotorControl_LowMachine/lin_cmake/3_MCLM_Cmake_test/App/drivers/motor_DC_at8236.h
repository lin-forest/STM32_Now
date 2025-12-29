// #ifndef __MOTOR_DC_AT8236_H
// #define __MOTOR_DC_AT8236_H

// #include "app_includes.h"

// typedef struct
// {
//     TIM_HandleTypeDef *PwmTimer;
//     uint32_t Channel1;
//     uint32_t Channel2;
//     uint16_t MaxPwm;
//     uint16_t MinPwm; // Minimum PWM value to overcome static friction
//     int16_t MaxSpeed;
// } At8236_Motor_t;

// typedef enum
// {
//     AT8236_STOP_COAST, // 滑行
//     AT8236_STOP_BRAKE  // 刹车
// } At8236_MotorStopMode_t;

// void At8236_Motor_Init(At8236_Motor_t *Motor, TIM_HandleTypeDef *PwmTimer, uint32_t Channel1, uint32_t Channel2, uint16_t MaxPwm, int16_t MaxSpeed);
// void At8236_Motor_SetSpeed(At8236_Motor_t *Motor, int16_t Speed);
// void At8236_Motor_Stop(At8236_Motor_t *Motor, At8236_MotorStopMode_t StopMode);

// #endif // __MOTOR_DC_AT8236_H