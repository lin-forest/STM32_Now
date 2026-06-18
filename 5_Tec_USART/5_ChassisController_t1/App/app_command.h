#ifndef APP_COMMAND_H
#define APP_COMMAND_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "app_config.h"

/* ===================== 命令消息结构体 ===================== */
/* 注意：命令类型枚举 Command_ID_t 定义在 app_config.h 中 */
typedef struct {
    Command_ID_t type;           // 命令类型
    int16_t  value;              // 参数值（如速度值）
    uint8_t  motor_id;           // 目标电机 ID，0xFF=广播
    uint32_t can_id;             // 关联的 CAN ID（用于回传/日志）
    uint8_t  raw_data[8];        // 原始数据（透传用）
    uint8_t  raw_len;            // 原始数据长度
} App_CommandMsg_t;

#endif /* APP_COMMAND_H */
