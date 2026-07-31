#ifndef __APP_INCLUDES_H__
#define __APP_INCLUDES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== STM32 HAL ===================== */
#include "main.h"
#include "stm32f1xx_hal.h"

#include "usart.h"      // huart1
#include "gpio.h"
#include "dma.h"

/* ===================== Application Config ===================== */
#include "app_config.h"

/* ===================== Services ===================== */
#include "echo_service.h"

/* ===================== C STD ===================== */
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
}
#endif

#endif /* __APP_INCLUDES_H__ */
