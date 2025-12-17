#ifndef __APP_TASKS_H__
#define __APP_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== FreeRTOS ===================== */
#include "cmsis_os.h"

/* ===================== Drivers ===================== */
#include "tb6612_DC.h"

/* ===================== Task Handles ===================== */
extern osThreadId_t MotorControl_TaHandle;
extern osThreadId_t Encoder_TaHandle;
extern osThreadId_t Logger_TaHandle;
extern osThreadId_t Command_TaHandle;
extern osThreadId_t Heartbeat_TaHandle;

/* ===================== Queues ===================== */
extern osMessageQueueId_t CommandQueueHandle;
extern osMessageQueueId_t AckQueueHandle;
extern osMessageQueueId_t MotorQueueHandle;

/* ===================== Global Objects（不冲突） ===================== */
extern Motor_t motor1;
extern uint8_t tx_buf[64];

/* ===================== Task Body Prototypes（你缺的关键部分） ===================== */
/* 这些函数就是你在各个 .c 文件中真正写逻辑的函数 */
void MotorControl_Task(void *argument);
void Encoder_Task(void *argument);
void Logger_Task(void *argument);
void Command_Task(void *argument);
void Heartbeat_Task(void *argument);
void Ack_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif



// #ifndef __APP_TASKS_H__
// #define __APP_TASKS_H__

// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "cmsis_os.h"
// #include "tb6612_DC.h"

// // ========== Task Handles ==========
// extern osThreadId_t MotorControl_TaHandle;
// extern osThreadId_t Encoder_TaHandle;
// extern osThreadId_t Logger_TaHandle;
// extern osThreadId_t Command_TaHandle;
// extern osThreadId_t Heartbeat_TaHandle;

// // ========== Queue ==========
// extern osMessageQueueId_t CommandQueueHandle;

// // ========== Global Objects ==========
// extern Motor_t motor1;

// // ========== logger buffer ==========
// extern uint8_t tx_buf[64];

// #ifdef __cplusplus
// }
// #endif

// #endif
