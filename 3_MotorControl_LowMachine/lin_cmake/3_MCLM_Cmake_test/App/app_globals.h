#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "pid.h"

/* ===================== Global Motor Status ===================== */
// A generic structure to hold the state of the active motor.
// This decouples the encoder and other tasks from specific motor drivers.
typedef struct {
    float    target_logic_speed;    // The desired speed (-100 to 100)
    float    current_logic_speed;   // The actual measured speed
    int32_t  current_ticks;         // The raw encoder ticks in the last period
    int16_t  pwm_output;            // The current PWM value applied to the motor
} MotorStatus_t;

extern MotorStatus_t g_motor_status;

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
extern uint8_t tx_buf[64];
extern PID_Controller motor_pid;


#ifdef __cplusplus
}
#endif

#endif /* __APP_GLOBALS_H__ */
