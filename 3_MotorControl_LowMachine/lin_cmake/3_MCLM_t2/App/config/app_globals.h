#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "pid.h"
#include "motor_DC_tb6612.h"

/* ===================== Global Motor Status ===================== */
// A generic structure to hold the state of the active motor.
// This decouples the encoder and other tasks from specific motor drivers.
typedef struct {
    TB6612_Motor_t hardware;       // 硬件驱动句柄（包含引脚、定时器等）
    PID_Controller pid;            // PID 控制器实例
    
    // 统一的状态反馈与控制量
    float    target_logic_speed;    // 目标逻辑速度 (-100 to 100)
    float    current_logic_speed;   // 实际测量速度
    int32_t  current_ticks;         // 编码器原始计数值
    int16_t  pwm_output;            // 当前输出的 PWM 值
} Motor_t;

// 支持多电机实例
#define MOTOR_COUNT 1 
extern Motor_t g_motors[MOTOR_COUNT];
extern volatile uint8_t g_logger_enabled;  // 0=停止发送, 1=发送实时数据

/* ===================== Task Handles ===================== */
extern osThreadId_t MotorControl_TaHandle;
extern osThreadId_t Encoder_TaHandle;
extern osThreadId_t Logger_TaHandle;
extern osThreadId_t Command_TaHandle;
extern osThreadId_t Heartbeat_TaHandle;
extern osThreadId_t Ack_TaHandle;

/* ===================== Queues ===================== */
extern osMessageQueueId_t CommandQueueHandle;
extern osMessageQueueId_t AckQueueHandle;
extern osMessageQueueId_t MotorQueueHandle;

/* ===================== Mutexes ===================== */
extern osMutexId_t motor0_mutexHandle;

/* ===================== Global Objects ===================== */
extern uint8_t tx_buf[64];


#ifdef __cplusplus
}
#endif

#endif /* __APP_GLOBALS_H__ */
