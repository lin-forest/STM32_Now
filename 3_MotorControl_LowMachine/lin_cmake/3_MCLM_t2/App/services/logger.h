

#ifndef __LOGGER_H
#define __LOGGER_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "command.h" // 如果 Logger 需要使用 CommandMsg_t 等，则保留
#include <stdint.h>

#define LOGGER_UART_RX_BUF_LEN 32
#define LOGGER_UART_TX_BUF_LEN 64 // 增加发送缓冲区长度，以支持更长的日志信息

typedef enum {
    LOGGER_OK = 0,
    LOGGER_BUSY,
    LOGGER_ERROR
} LoggerStatus_t;

// App/services/logger.h 新增
// 既使用web-gpt,web-gemini,codex,gemini-assist,trae-gemini2.5pro,claude等等，开始独立修改代码，进行个人补足与重构_20260427-11:11
typedef struct {
    uint8_t motor_id;           // 电机ID（0或1）
    int32_t current_ticks;
    float   target_logic_speed;
    int16_t pwm_output;
    uint32_t timestamp_ms;
} LogMotorData_t;

// 初始化日志模块（可扩展为多种日志方式）
LoggerStatus_t Logger_Init(void);

// 日志输出（可扩展为带级别、格式化等）
LoggerStatus_t Logger_Print(const char *msg);
LoggerStatus_t Logger_Printf(const char *fmt, ...);

// 串口接收中断回调 (如果需要从 UART 接收数据并处理，可以保留，但通常日志模块只负责输出)
// void Logger_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 可选：注册回调、查询状态等接口

#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H */

