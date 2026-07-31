#include "app_includes.h"

AirCommand_t CAN_CMD_Decode(uint8_t *data, uint16_t stdid)
{
    AirCommand_t cmd = {0};

    /* 只响应气动系统命令 ID */
    if (stdid != CAN_AIR_CMD_STDID)
        return cmd;

    cmd.pump1_pwm = data[CMD_IDX_PUMP1_PWM];
    cmd.pump2_pwm = data[CMD_IDX_PUMP2_PWM];
    cmd.valve1_on = data[CMD_IDX_VALVE1] ? 1 : 0;
    cmd.valve2_on = data[CMD_IDX_VALVE2] ? 1 : 0;

    /* 钳位 */
    if (cmd.pump1_pwm > PWM_MAX_DUTY) cmd.pump1_pwm = PWM_MAX_DUTY;
    if (cmd.pump2_pwm > PWM_MAX_DUTY) cmd.pump2_pwm = PWM_MAX_DUTY;

    return cmd;
}

void CAN_CMD_EncodeStatus(uint8_t *data, AirCommand_t *cmd, uint8_t flags)
{
    data[STA_IDX_PUMP1_PWM] = cmd->pump1_pwm;
    data[STA_IDX_PUMP2_PWM] = cmd->pump2_pwm;
    data[STA_IDX_VALVE1]    = cmd->valve1_on ? 1 : 0;
    data[STA_IDX_VALVE2]    = cmd->valve2_on ? 1 : 0;
    data[STA_IDX_FLAGS]     = flags;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
}

HAL_StatusTypeDef CAN_SendStatus(CAN_HandleTypeDef *hcan, AirCommand_t *cmd, uint8_t flags)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = CAN_AIR_STATUS_STDID;
    txHeader.ExtId = 0;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    CAN_CMD_EncodeStatus(txData, cmd, flags);

    return HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);
}
