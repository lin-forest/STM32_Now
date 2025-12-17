#ifndef __tb6612_DC_H
#define __tb6612_DC_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机停止模式
 */
typedef enum {
    MOTOR_STOP_COAST = 0,   // 悬空停止（两相低电平）
    MOTOR_STOP_BRAKE        // 快速刹车（两相高电平）
} MotorStopMode_t;

/**
 * @brief 电机结构体定义
 */
typedef struct {
    // --- 硬件相关 ---
    TIM_HandleTypeDef *htim;  // PWM 定时器
    uint32_t Channel;         // PWM 通道
    GPIO_TypeDef *IN1_Port;   // IN1 引脚端口
    uint16_t IN1_Pin;         // IN1 引脚
    GPIO_TypeDef *IN2_Port;   // IN2 引脚端口
    uint16_t IN2_Pin;         // IN2 引脚
    GPIO_TypeDef *EN_Port;    // 使能端口（可选，没有则 NULL）
    uint16_t EN_Pin;          // 使能引脚

    // --- 参数配置 ---
    uint16_t MaxPWM;          // 最大 PWM 值 (CubeMX TIM ARR)
    int16_t MaxSpeed;         // 最大速度（用户逻辑范围，例 ±1000）
    uint16_t DeadZone;        // 死区PWM（避免小值电机不动）
    uint8_t Polarity;         // 极性(0=默认,1=反转)
    MotorStopMode_t StopMode; // 停止模式

    // --- 状态量 ---
    int16_t current_speed;   // 实际速度（编码器反馈）
    int16_t target_speed;    // 目标速度
    int16_t pwm_output;      // 当前 PWM 输出值（新增）
} Motor_t;

// ================= API 函数声明 =================

/**
 * @brief 初始化电机
 */
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t Channel,
                GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
                uint16_t MaxPWM, int16_t MaxSpeed, uint16_t DeadZone,
                uint8_t Polarity, MotorStopMode_t StopMode);

/**
 * @brief 设置电机速度（开环PWM控制）
 * @param speed - 范围：[-MaxSpeed, MaxSpeed]
 */
void Motor_SetSpeed(Motor_t *motor, int16_t speed);

/**
 * @brief 电机停止（根据StopMode选择悬空/刹车）
 */
void Motor_Stop(Motor_t *motor);

/**
 * @brief 电机紧急制动（可选扩展接口）
 */
static inline void Motor_EmergencyStop(Motor_t *motor) {
    HAL_TIM_PWM_Stop(motor->htim, motor->Channel);
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
}

#ifdef __cplusplus
}
#endif

// #endif /* __MOTOR_H */
// /* === 添加全局电机对象声明（关键！） === */
extern Motor_t motor1;

#endif  // __tb6612_DC_H



// #ifndef __tb6612_DC_H
// #define __tb6612_DC_H

// #include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_tim.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// /**
//  * @brief 电机停止模式
//  */
// typedef enum {
//     MOTOR_STOP_COAST = 0,   // 悬空停止（两相低电平）
//     MOTOR_STOP_BRAKE        // 快速刹车（两相高电平）
// } MotorStopMode_t;

// /**
//  * @brief 电机结构体定义
//  */
// typedef struct {
//     // --- 硬件相关 ---
//     TIM_HandleTypeDef *htim;  // PWM 定时器
//     uint32_t Channel;         // PWM 通道
//     GPIO_TypeDef *IN1_Port;   // IN1 引脚端口
//     uint16_t IN1_Pin;         // IN1 引脚
//     GPIO_TypeDef *IN2_Port;   // IN2 引脚端口
//     uint16_t IN2_Pin;         // IN2 引脚
//     GPIO_TypeDef *EN_Port;    // 使能端口（可选，没有则 NULL）
//     uint16_t EN_Pin;          // 使能引脚

//     // --- 参数配置 ---
//     uint16_t MaxPWM;          // 最大 PWM 值 (CubeMX TIM ARR)
//     int16_t MaxSpeed;         // 最大速度（用户逻辑范围，例 ±1000）
//     uint16_t DeadZone;        // 死区PWM（避免小值电机不动）
//     uint8_t Polarity;         // 极性(0=默认,1=反转)
//     MotorStopMode_t StopMode; // 停止模式

//     // --- 状态量 ---
//     int16_t speed_current;   // 实际速度（编码器反馈）
//     int16_t speed_target;    // 目标速度
//     int16_t pwm_output;      // 当前 PWM 输出值（新增）
// } Motor_t;

// // ================= API 函数声明 =================

// /**
//  * @brief 初始化电机
//  */
// void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t Channel,
//                 GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
//                 GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
//                 GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
//                 uint16_t MaxPWM, int16_t MaxSpeed, uint16_t DeadZone,
//                 uint8_t Polarity, MotorStopMode_t StopMode);

// /**
//  * @brief 设置电机速度（开环PWM控制）
//  * @param speed - 范围：[-MaxSpeed, MaxSpeed]
//  */
// void Motor_SetSpeed(Motor_t *motor, int16_t speed);

// /**
//  * @brief 电机停止（根据StopMode选择悬空/刹车）
//  */
// void Motor_Stop(Motor_t *motor);

// /**
//  * @brief 电机紧急制动（可选扩展接口）
//  */
// static inline void Motor_EmergencyStop(Motor_t *motor) {
//     HAL_TIM_PWM_Stop(motor->htim, motor->Channel);
//     HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
// }

// #ifdef __cplusplus
// }
// #endif

// /* === 添加全局电机对象声明（关键！） === */
// extern Motor_t motor1;

// #endif  // __tb6612_DC_H
