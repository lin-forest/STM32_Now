#ifndef __COMMAND_H
#define __COMMAND_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 命令类型定义 ===================== */

typedef enum {
    CMD_NONE = 0,      // 无效命令
    CMD_FORWARD,       // 前进（具体速度由电机控制任务决定）
    CMD_REVERSE,       // 后退
    CMD_STOP,          // 停止
    CMD_SET_SPEED,      // 设置速度：例如 "S500"

    CMD_LIST_STATUS,     // ← 新增：ls

    // CAN 命令类型
    CAN_CMD_SET_SPEED, // CAN 设置速度命令
    CAN_CMD_STOP,      // CAN 停止命令
} CommandType_t;

/* ===================== 命令消息结构 ===================== */

typedef struct {
    CommandType_t type;    // 命令类型
    int16_t value;         // 参数（如速度值），对无参数命令无效
} CommandMsg_t;


// ACK机制暂时无用
/* ===================== 命令执行反馈（ACK） ===================== */
typedef struct {
    CommandType_t type;   // 对应的命令类型
    int16_t value;        // 可选：速度 / 参数
    uint8_t ok;           // 1 = 成功执行，0 = 失败
} AckMsg_t;


/* ===================== 协议说明 ===================== */
/*
 * 字符串协议格式：
 *   "Sxxx"    → CMD_SET_SPEED, value = xxx
 *   "F"       → CMD_FORWARD
 *   "R"       → CMD_REVERSE
 *   "X"       → CMD_STOP
 *
 * 数字格式支持正/负，如 "S-300"
 */

/* ===================== 函数声明 ===================== */

// 字符串解析（主函数）
// 输入:    字符串，例如 "S500"
// 返回:    CommandMsg_t结构体
CommandMsg_t Command_ParseString(const char *cmdStr);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_H */