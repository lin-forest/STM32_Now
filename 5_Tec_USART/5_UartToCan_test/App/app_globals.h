#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H

/* Includes ------------------------------------------------------------------*/
// 这里只包含定义这些句柄类型所必需的头文件
#include "can.h"
#include "usart.h"
#include "cmsis_os.h"

/* Extern declarations of handles defined elsewhere (e.g., in freertos.c, can.c) */

// HAL Driver Handles
extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// FreeRTOS Object Handles
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t uartToCanQueueHandle;

#endif /* APP_GLOBALS_H */
