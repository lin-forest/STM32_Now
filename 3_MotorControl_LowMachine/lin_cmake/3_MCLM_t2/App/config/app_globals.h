#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "pid.h"
#include "motor_DC_tb6612.h"
#include "logger.h"

/* ===================== Motor Status Flags ===================== */
#define MOTOR_FLAG_STALL      0x01U   // 堵转：setpoint != 0 但 speed ≈ 0
#define MOTOR_FLAG_SATURATED  0x02U   // 饱和：PWM 已达上限但无法达到目标速度

/* ===================== Global Motor Status ===================== */
// A generic structure to hold the state of the active motor.
// This decouples the encoder and other tasks from specific motor drivers.
typedef struct {
    TB6612_Motor_t hardware;       // 硬件驱动句柄（包含引脚、定时器等）
    PID_Controller pid;            // PID 控制器实例

    // 统一的状态反馈与控制量
    float    target_logic_speed;    // 目标逻辑速度 (-100 to 100)
    float    current_logic_speed;   // 实际测量速度
    int32_t  current_ticks;         // 编码器原始计数值（每周期差值）
    int32_t  accumulated_ticks;     // 编码器累计计数值（绝对位置）
    int16_t  pwm_output;            // 当前输出的 PWM 值

    // 状态检测
    uint8_t  flags;                // 电机状态标志 (MOTOR_FLAG_*)
    uint8_t  stall_counter;        // 连续堵转周期计数
} Motor_t;

// 支持多电机实例
#define MOTOR_COUNT 2
extern Motor_t g_motors[MOTOR_COUNT];
extern volatile uint8_t g_logger_enabled;  // 0=停止发送, 1=发送实时数据

/* ===================== Task Handles ===================== */
extern osThreadId_t MotorControl_TaHandle;   // 电机0（转向）控制任务
extern osThreadId_t Encoder_TaHandle;         // 电机0 编码器任务
extern osThreadId_t Logger_TaHandle;
extern osThreadId_t Command_TaHandle;
extern osThreadId_t Heartbeat_TaHandle;
extern osThreadId_t Ack_TaHandle;
extern osThreadId_t MotorControl1_THandle;   // 电机1（动力）控制任务
extern osThreadId_t Encoder1_THandle;         // 电机1 编码器任务

/* ===================== Queues ===================== */
extern osMessageQueueId_t CommandQueueHandle;
extern osMessageQueueId_t AckQueueHandle;
extern osMessageQueueId_t MotorQueueHandle;   // 电机0 专属队列（原 MotorQueue）
extern osMessageQueueId_t MotorQueue1Handle;  // 电机1 专属队列
extern osMessageQueueId_t LogQueueHandle;      // 日志队列

/* ===================== Mutexes ===================== */
extern osMutexId_t motor0_mutexHandle;        // 保护 g_motors[0]
extern osMutexId_t motor1_mutexHandle;        // 保护 g_motors[1]

/* ===================== Semaphores ===================== */
extern osSemaphoreId_t uart_rx_semaphoreHandle;
extern osSemaphoreId_t can_rx_semaphoreHandle;
extern osSemaphoreId_t uart1_dma_semHandle;   // 用于 Logger UART1 DMA 发送完成的信号量

/* ===================== Global Objects ===================== */
extern uint8_t tx_buf[64];


#ifdef __cplusplus
}
#endif

#endif /* __APP_GLOBALS_H__ */
