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
#define MOTOR_PID_KP 0.4584f
#define MOTOR_PID_KI 17.66f
#define MOTOR_PID_KD 0.002976f
#define MOTOR_PID_INTEGRAL_LIMIT 500.0f
#define MOTOR_PID_OUTPUT_LIMIT 100.0f

// Motor Initialization Parameters
#define MOTOR1_TIM_HANDLE           &htim3
#define MOTOR1_TIM_CHANNEL          TIM_CHANNEL_1
#define MOTOR1_IN1_PORT             GPIOB
#define MOTOR1_IN1_PIN              GPIO_PIN_0
#define MOTOR1_IN2_PORT             GPIOB
#define MOTOR1_IN2_PIN              GPIO_PIN_1
#define MOTOR1_EN_PORT              GPIOB
#define MOTOR1_EN_PIN               GPIO_PIN_10
#define MOTOR1_MAX_SPEED_LOGIC      100
#define MOTOR1_MAX_PWM_OUTPUT       100
#define MOTOR1_MIN_PWM_OUTPUT       10
#define MOTOR1_DEAD_ZONE            0
#define MOTOR1_STOP_MODE            TB6612_MOTOR_STOP_BRAKE // Updated to use TB6612_MOTOR_STOP_BRAKE

// ACK Message Buffer Size
#define ACK_MSG_BUF_SIZE            128 // 定义ACK消息缓冲区的最大大小

#endif // APP_CONFIG_H