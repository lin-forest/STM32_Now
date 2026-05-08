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
extern osMutexId_t        uart1_tx_mutexHandle;  // 保护 huart1 的多任务并发发送
extern osSemaphoreId_t    uart1_tx_semHandle;     // DMA TX 完成信号量（ISR → Task）
extern osEventFlagsId_t   uart1_rx_eventHandle;  // ISR 通知 ProtocolParser 有新数据

#define UART1_RX_FLAG  0x01U  // uart1_rx_eventHandle 使用的标志位

// ISR 中递增的 CAN TX 完成计数器 (volatile: ISR 写，Task 读)
extern volatile uint32_t can_tx_done_cnt;

#endif /* APP_GLOBALS_H */
