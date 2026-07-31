#ifndef __APP_INCLUDES_H__
#define __APP_INCLUDES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== STM32 HAL ===================== */
#include "main.h"
#include "stm32f1xx_hal.h"

#include "tim.h"        // htim2
#include "can.h"        // hcan
#include "gpio.h"

/* ===================== FreeRTOS (CMSIS V2) ===================== */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"

/* ===================== Application Config ===================== */
#include "app_config.h"

/* ===================== Drivers ===================== */
#include "ibt4_switch.h"

/* ===================== Services ===================== */
#include "can_protocol.h"

/* ===================== C STD ===================== */
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
}
#endif

#endif
