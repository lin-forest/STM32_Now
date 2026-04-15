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

#include "app_globals.h"

/* ===================== Task Body Prototypes（你缺的关键部分） ===================== */
/* 这些函数就是你在各个 .c 文件中真正写逻辑的函数 */
void MotorControl_Task(void *argument);
void Encoder_Task(void *argument);
void Logger_Task(void *argument);
void Command_Task(void *argument);
void Heartbeat_Task(void *argument);
void Ack_Task(void *argument);
void TB6612_DC_Task(void *argument);
void AT8236_DC_Task(void *argument);
void Motor_PID_Init(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif