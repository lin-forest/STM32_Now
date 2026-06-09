#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* =================================================================================
 *   1. 电机驱动类型定义 (Motor Driver Type Definitions)
 * ================================================================================= */
#define MOTOR_DRIVER_TB6612 1
#define MOTOR_DRIVER_IBT4   3


/* =================================================================================
 *   2. 全局共享参数 (Global Shared Parameters)
 *      两电机共用，不随预设切换
 * ================================================================================= */
#define SPEED_LOGIC_MAX     100   // 统一的逻辑速度最大值 (例如 0-100)
#define PWM_MAX             7200  // PWM最大值, 对应定时器的ARR寄存器值, 代表100%占空比
#define ENCODER_FILTER_ALPHA    0.1f  // 编码器IIR滤波系数，越小越平滑，响应越慢
#define MOTOR_CMD_DEFAULT_SPEED  50.0f  // CMD_FORWARD / CMD_REVERSE 的默认逻辑速度 (占满量程 50%)


/* =================================================================================
 *   3. 电机参数预设选择
 *      默认预设 (MOTOR_CFG_DEFAULT) — 保留原有电机配置和 PID 参数
 *      新电机预设 (MOTOR_CFG_NEW)   — 实验新参数，互不干扰
 * ================================================================================= */
#define MOTOR_CFG_DEFAULT  1
#define MOTOR_CFG_NEW      2
#define MOTOR_CFG_2IBT4    3

#define MOTOR_CFG_SET   3

#if   MOTOR_CFG_SET == MOTOR_CFG_DEFAULT
  #include "app_motor_cfg_default.h"
#elif MOTOR_CFG_SET == MOTOR_CFG_NEW
  #include "app_motor_cfg_new.h"
#elif MOTOR_CFG_SET == MOTOR_CFG_2IBT4
  #include "app_motor_cfg_2ibt4.h"
#else
  #error "MOTOR_CFG_SET must be MOTOR_CFG_DEFAULT (1), MOTOR_CFG_NEW (2), or MOTOR_CFG_2IBT4 (3)"
#endif


/* =================================================================================
 *   4. CAN 通信参数
 * ================================================================================= */

/* ------------------- ACK消息缓冲区 ------------------- */
#define ACK_MSG_BUF_SIZE            128

/* ------------------- CAN总线配置 ------------------- */
/* CAN ID 组选择：改下面这个数字即可切换
 *   1 = 0x125/0x126 系列
 *   2 = 0x123/0x124 系列
 */
#define CAN_ID_GROUP  1

#if CAN_ID_GROUP == 1
    #define CAN_MOTOR_TURN_CMD_STDID             0x125   // 方向电机控制指令的CAN ID
    #define CAN_MOTOR_TURN_CMD_STATUS_STDID      0x225   // 方向上层控制电机状态的CAN ID
    #define CAN_MOTOR_TURN_STATUS_STDID          0x325   // 方向电机状态反馈的CAN ID
    #define CAN_MOTOR_POWER_CMD_STDID            0x126   // 动力电机控制指令的CAN ID
    #define CAN_MOTOR_POWER_CMD_STATUS_STDID     0x226   // 动力上层控制电机状态的CAN ID
    #define CAN_MOTOR_POWER_STATUS_STDID         0x326   // 动力电机状态反馈的CAN ID
#elif CAN_ID_GROUP == 2
    #define CAN_MOTOR_TURN_CMD_STDID             0x123   // 方向电机控制指令的CAN ID
    #define CAN_MOTOR_TURN_CMD_STATUS_STDID      0x223   // 方向上层控制电机状态的CAN ID
    #define CAN_MOTOR_TURN_STATUS_STDID          0x323   // 方向电机状态反馈的CAN ID
    #define CAN_MOTOR_POWER_CMD_STDID            0x124   // 动力电机控制指令的CAN ID
    #define CAN_MOTOR_POWER_CMD_STATUS_STDID     0x224   // 动力上层控制电机状态的CAN ID
    #define CAN_MOTOR_POWER_STATUS_STDID         0x324   // 动力电机状态反馈的CAN ID
#else
    #error "CAN_ID_GROUP must be 1 (0x125/0x126) or 2 (0x123/0x124)"
#endif

#define CAN_CMD_SET_SPEED_T2            0x11    // 新的设置速度命令
#define CAN_CMD_REVERSE_BYTE            0x02    // 独立倒转命令字节（宏，避免与枚举 CMD_REVERSE=2 在 switch 中混用）
#define CAN_CMD_QUERY_STATUS            0x01    // 查询状态命令 (兼容旧协议)
#define CAN_CMD_LOG_START               0x04    // 开始发送实时电机数据
#define CAN_CMD_LOG_STOP                0x05    // 停止发送实时电机数据

// 全车停止
#define CAN_CMD_STOP_STDID              0x101   // 全车停止命令的CAN ID
// 全车转向命令
#define CAN_CMD_TURN_STDID              0x102   // 转向命令的CAN ID
// 全车动力命令
#define CAN_CMD_POWER_STDID             0x103   // 动力命令的CAN ID

/* CAN 硬件初始化 */
#define CAN_PRESCALER               4
#define CAN_MODE                    CAN_MODE_NORMAL
#define CAN_SYNC_JUMP_WIDTH         CAN_SJW_1TQ
#define CAN_TIME_SEG1               CAN_BS1_13TQ
#define CAN_TIME_SEG2               CAN_BS2_4TQ
#define CAN_TIME_TRIGGERED_MODE     DISABLE
#define CAN_AUTO_BUS_OFF            ENABLE
#define CAN_AUTO_WAKE_UP            DISABLE
#define CAN_AUTO_RETRANSMISSION     ENABLE
#define CAN_RECEIVE_FIFO_LOCKED     DISABLE
#define CAN_TRANSMIT_FIFO_PRIORITY  ENABLE

/* CAN 过滤器配置 */
#define CAN_FILTER_BANK             0
#define CAN_FILTER_MODE             CAN_FILTERMODE_IDMASK
#define CAN_FILTER_SCALE            CAN_FILTERSCALE_32BIT
#define CAN_FILTER_ID_HIGH          0x0000
#define CAN_FILTER_ID_LOW           0x0000
#define CAN_FILTER_MASK_ID_HIGH     0x0000
#define CAN_FILTER_MASK_ID_LOW      0x0000
#define CAN_FILTER_FIFO             CAN_RX_FIFO0
#define CAN_FILTER_ACTIVATION       ENABLE
#define CAN_SLAVE_START_FILTER_BANK 14

/* ================= CAN Protocol Definitions ================= */
// 定义CAN数据帧中不同数据字段的索引
#define CAN_DATA_INDEX_CMD      0   // 命令类型在数据帧中的索引
#define CAN_DATA_INDEX_SPEED    1   // 速度值在数据帧中的索引



#endif // __APP_CONFIG_H__
