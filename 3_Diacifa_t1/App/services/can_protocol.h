#ifndef __CAN_PROTOCOL_H__
#define __CAN_PROTOCOL_H__

#include <stdint.h>
#include "stm32f1xx_hal_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =================================================================================
 *  气动系统 CAN 协议定义
 *
 *  CAN ID 分配：
 *    0x141 (RX) — 控制命令：气泵PWM + 电磁阀开关
 *    0x341 (TX) — 状态上报：当前PWM/阀状态/故障码
 *
 *  命令帧 (8 bytes, CAN 标准帧 11bit ID)：
 *    [0] pump1_pwm:  0-100  (0=停止)
 *    [1] pump2_pwm:  0-100  (0=停止)
 *    [2] valve1:     0=失电, 1=得电
 *    [3] valve2:     0=失电, 1=得电
 *    [4-7] reserved
 *
 *  状态帧 (8 bytes)：
 *    [0] pump1_pwm:  当前实际 PWM
 *    [1] pump2_pwm:  当前实际 PWM
 *    [2] valve1:     当前阀状态
 *    [3] valve2:     当前阀状态
 *    [4] flags:      故障标志 (AIR_FLAG_*)
 *    [5-7] reserved
 * ================================================================================= */

/**
 * @brief  气动系统控制命令结构体
 */
typedef struct {
    uint8_t pump1_pwm;      // 气泵1 目标PWM (0-100)
    uint8_t pump2_pwm;      // 气泵2 目标PWM (0-100)
    uint8_t valve1_on;      // 电磁阀1 (0/1)
    uint8_t valve2_on;      // 电磁阀2 (0/1)
} AirCommand_t;

/**
 * @brief  解码 CAN 接收数据帧为 AirCommand
 * @param  data       CAN 数据 (8 bytes)
 * @param  stdid      CAN 标准帧 ID (用于校验)
 * @return AirCommand_t  解码后的命令
 */
AirCommand_t CAN_CMD_Decode(uint8_t *data, uint16_t stdid);

/**
 * @brief  编码状态帧数据
 * @param  data       写入的 8 字节缓冲区
 * @param  cmd        当前命令状态
 * @param  flags      故障标志
 */
void CAN_CMD_EncodeStatus(uint8_t *data, AirCommand_t *cmd, uint8_t flags);

/**
 * @brief  通过 CAN 发送状态帧
 * @param  hcan       CAN 句柄
 * @param  cmd        当前命令状态
 * @param  flags      故障标志
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_SendStatus(CAN_HandleTypeDef *hcan, AirCommand_t *cmd, uint8_t flags);

#ifdef __cplusplus
}
#endif

#endif // __CAN_PROTOCOL_H__
