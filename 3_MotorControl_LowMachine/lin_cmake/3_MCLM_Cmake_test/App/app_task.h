#ifndef __APP_TASKS_H__
#define __APP_TASKS_H__

#include "cmsis_os2.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ===================== FreeRTOS ===================== */
// #include "cmsis_os.h"

// /* ===================== Drivers ===================== */
#include "motor_DC_tb6612.h"
#include "pid.h"
// #include "motor_BLDC.h"
// #include "motor_DC_ibt4.h"

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
extern osMessageQueueId_t CanMotorCmdQueueHandle;

// 在 CubeMX 中创建名为 motor_mutex 的互斥锁后，添加此声明
extern osMutexId_t motor_mutexHandle;

/* ===================== Global Objects（不冲突） ===================== */
extern TB6612_Motor_t tb6612_motor1; // Changed from Motor_t motor1 to TB6612_Motor_t tb6612_motor1
extern uint8_t tx_buf[64];
extern PID_Controller motor_pid;

/* ===================== Task Body Prototypes（你缺的关键部分） ===================== */
/* 这些函数就是你在各个 .c 文件中真正写逻辑的函数 */
void MotorControl_Task(void *argument);
void Encoder_Task(void *argument);
void Logger_Task(void *argument);
void Command_Task(void *argument);
void Heartbeat_Task(void *argument);
void Ack_Task(void *argument);
void tb6612_DC_Task(void *argument);
// void at8236_DC_Task(void *argument);
void Motor_PID_Init(void);

#ifdef __cplusplus
}
#endif

#endif