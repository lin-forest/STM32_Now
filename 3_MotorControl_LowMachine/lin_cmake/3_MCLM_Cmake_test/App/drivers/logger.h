
#ifndef __UART_APP_H
#define __UART_APP_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "command.h"

// 串口接收缓冲区长度
#define UART2_RX_BUF_LEN 32

// 串口初始化函数
void UART_App_Init(void);

// 串口发送函数
void UART2_Print(const char *msg);

// 串口接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif




// #ifndef __LOGGER_H
// #define __LOGGER_H

// #include "stm32f1xx_hal.h"
// #include "cmsis_os.h"
// #include "command.h"

// #define LOGGER_UART_RX_BUF_LEN 32
// #define LOGGER_UART_TX_BUF_LEN 64

// typedef enum {
//     LOGGER_OK = 0,
//     LOGGER_BUSY,
//     LOGGER_ERROR
// } LoggerStatus_t;

// // 初始化日志模块（可扩展为多种日志方式）
// LoggerStatus_t Logger_Init(void);

// // 日志输出（可扩展为带级别、格式化等）
// LoggerStatus_t Logger_Print(const char *msg);
// LoggerStatus_t Logger_Printf(const char *fmt, ...);

// // 串口接收中断回调
// void Logger_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// // 可选：注册回调、查询状态等接口

// #endif

