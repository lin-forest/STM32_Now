#ifndef __APP_INCLUDES_H__
#define __APP_INCLUDES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== STM32 HAL ===================== */
#include "main.h"
#include "stm32f1xx_hal.h"

#include "tim.h"        // htim2, htim3等
#include "usart.h"      // huart1
#include "gpio.h"

/* ===================== FreeRTOS (CMSIS V2) ===================== */
// #include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ===================== Application Config ===================== */
#include "app_config.h"


/* ===================== Drivers & Modules ===================== */
// #include "tb6612_DC.h"      // 电机结构体 Motor_t        // app_task.h
#include "command.h"        // 命令解析模块
#include "logger.h"         // Logger 函数
#include "app_task.h"       // 全局任务句柄 & motor1
#include "speed_map.h"

#include "motor_DC_tb6612.h"
#include "motor_BLDC.h"
#include "motor_DC_ibt4.h"



/* ===================== C STD ===================== */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
}
#endif

#endif


// #ifndef __APP_INCLUDES_H__
// #define __APP_INCLUDES_H__

// // #include "cmsis_os.h"


// #endif