#ifndef __CAN_FILTER_H
#define __CAN_FILTER_H

#include "stdint.h"
#include "command.h"

typedef enum {
    CAN_FILTER_ACCEPT = 0,
    CAN_FILTER_REJECT
} CAN_FilterResult_t;

/**
 * @brief  检查 CAN ID + 命令字节组合是否在白名单内
 * @param  stdId   标准帧 ID
 * @param  cmdByte 命令字节 (rxData[0])
 * @return CAN_FILTER_ACCEPT 或 CAN_FILTER_REJECT
 */
CAN_FilterResult_t CAN_Filter_Accept(uint32_t stdId, uint8_t cmdByte);

/**
 * @brief  根据 CAN ID 获取目标电机 ID
 * @param  stdId 标准帧 ID
 * @return 0=转向电机, 1=动力电机, 0xFF=广播
 */
uint8_t CAN_Filter_GetMotorId(uint32_t stdId);

/**
 * @brief  命令字节 → CommandType_t 映射
 * @param  stdId   标准帧 ID (某些命令需要结合 ID 判断)
 * @param  cmdByte 命令字节
 * @return CommandType_t, 无效返回 CMD_NONE
 */
CommandType_t CAN_Filter_CmdByteToType(uint32_t stdId, uint8_t cmdByte);

/**
 * @brief  提取命令参数字节中的速度值
 * @param  stdId   标准帧 ID
 * @param  cmdByte 命令字节
 * @param  rxData  完整数据帧 (rxData[1] 为速度字段)
 * @return int16_t 速度值, 无效命令返回 0
 */
int16_t CAN_Filter_GetValue(uint32_t stdId, uint8_t cmdByte, const uint8_t rxData[8]);

#endif /* __CAN_FILTER_H */
