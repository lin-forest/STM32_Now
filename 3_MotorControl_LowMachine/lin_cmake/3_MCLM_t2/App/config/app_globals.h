#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "pid.h"
#include "app_config.h"   // 获取 MOTOR_COUNT 及 ACTIVE_MOTOR_DRIVER 等全局配置  [Fix #2]

/* =================================================================================
 *   驱动自适应 hardware 类型  [Fix #3]
 *   根据 ACTIVE_MOTOR_DRIVER 自动引入对应头文件并定义 MotorHW_t 类型别名
 * ================================================================================= */
#if   (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
    #include "motor_DC_tb6612.h"
    typedef TB6612_Motor_t MotorHW_t;
#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)
    #include "motor_DC_at8236.h"
    typedef AT8236_Motor_t MotorHW_t;
#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_IBT4)
    #include "motor_DC_ibt4.h"
    typedef IBT4_Motor_t   MotorHW_t;
#else
    #error "ACTIVE_MOTOR_DRIVER is not set to a recognized value!"
#endif

/* ===================== Global Motor Status ===================== */
// A generic structure to hold the state of the active motor.
// hardware 字段类型随 ACTIVE_MOTOR_DRIVER 自动切换，不再硬编码 TB6612。  [Fix #3]
typedef struct {
    MotorHW_t  hardware;             // 硬件驱动句柄（引脚、定时器等；类型随驱动选择自动切换）
    PID_Controller pid;              // PID 控制器实例

    // 统一的状态反馈与控制量
    int16_t    target_logic_speed;   // 目标逻辑速度 (-100 to 100)
    int16_t    measured_speed;       // 测量速度（正常模式为逻辑值，校准模式为原始增量）  [Fix #9]
    int16_t    current_ticks;        // 本周期编码器增量（有符号，正转为正）
    int32_t    encoder_count;        // 累积绝对计数（用于校准编码器线数，可复位）
    int16_t    pwm_output;           // 当前输出的 PWM 值
} Motor_t;

// MOTOR_COUNT 来自 app_config.h，此处不再重复定义  [Fix #2]
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
extern osMutexId_t motor_mutexHandle;

/* ===================== Global Objects ===================== */
// tx_buf 扩容至 128 字节：logger 最大帧约 62 B，加 null 及余量后 64 B 存在截断风险  [Fix #6]
extern uint8_t tx_buf[128];


#ifdef __cplusplus
}
#endif

#endif /* __APP_GLOBALS_H__ */
