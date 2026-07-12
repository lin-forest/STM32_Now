#ifndef __COMMAND_H__
#define __COMMAND_H__

#include <stdint.h>

/* ============================================================
 *  命令类型枚举
 * ============================================================ */
typedef enum {
    CMD_NONE = 0,
    CMD_SET_POSITION = 0x11,    // 绝对位置
    CMD_MULTI_JOINT  = 0x12,    // 多关节同步
    CMD_INCREMENT    = 0x21,    // 增量
    CMD_STOP         = 0x08,
} ArmCmdType_t;

/* ============================================================
 *  命令消息结构 (FreeRTOS 队列传递)
 * ============================================================ */
typedef struct {
    uint8_t  cmd_type;      // 命令类型
    uint8_t  joint_id;      // 关节 ID
    int16_t  value;         // 目标值
    uint8_t  bitmask;       // 多关节模式位掩码
    uint8_t  data[8];       // 原始 CAN 数据
} ArmCmdMsg_t;

/* ============================================================
 *  关节 ID 定义
 * ============================================================ */
#define JOINT_ID_J0          0   // DC 电机 (第三关节)
#define JOINT_ID_J1          1   // 舵机 J1
#define JOINT_ID_J2          2   // 舵机 J2
#define JOINT_ID_GRIPPER     3   // 夹爪
#define JOINT_ID_ALL         0xFF

/* 多关节位掩码 */
#define MASK_J0      0x01
#define MASK_J1      0x02
#define MASK_J2      0x04
#define MASK_GRIPPER 0x08

/* ============================================================
 *  CAN 消息结构
 * ============================================================ */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} CAN_Msg_t;

#endif // __COMMAND_H__
