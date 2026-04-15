#ifndef __COMMAND_H
#define __COMMAND_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 命令类型定义 ===================== */

typedef enum {
    CMD_NONE = 0,      // 无效命令
    CMD_FORWARD = 1,       // 前进（具体速度由电机控制任务决定）
    CMD_REVERSE = 2,       // 后退
    CMD_STOP = 3,          // 停止
    CMD_SET_SPEED = 4,      // 设置速度：例如 "S500"

    CMD_LIST_STATUS = 5,     // ls：查询状态（UART）
    CMD_QUERY_STATUS = 6,    // 查询状态（CAN 0x201帧）

    // CAN 命令类型
    CAN_CMD_SET_SPEED = 7, // CAN 设置速度命令
    CAN_CMD_STOP = 8,      // CAN 停止命令

    // 数据流控制
    CMD_LOG_START = 9,     // 开始发送实时电机数据
    CMD_LOG_STOP = 10,      // 停止发送实时电机数据
} CommandType_t;

/* ===================== 命令消息结构 ===================== */

typedef struct {
    CommandType_t type;    // 命令类型
    int16_t value;         // 参数（如速度值），对无参数命令无效
} CommandMsg_t;


/* ===================== 命令执行反馈（ACK） ===================== */
typedef struct {
    CommandType_t type;   // 对应的命令类型
    int16_t value;        // 可选：速度 / 参数
    uint8_t ok;           // 1 = 成功执行，0 = 失败
    int16_t current_logic_speed; // 新增：反馈当前逻辑速度
    int16_t pwm_output;          // 新增：反馈当前PWM输出值
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