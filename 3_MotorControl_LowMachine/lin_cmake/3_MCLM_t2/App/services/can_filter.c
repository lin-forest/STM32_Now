#include "can_filter.h"
#include "app_config.h"

/* ===================== 白名单表 ===================== */

typedef struct {
    uint32_t std_id;
    uint8_t  motor_id;           /* 0=转向, 1=动力, 0xFF=广播 */
    uint8_t  allowed_cmds[6];    /* 允许的命令字节列表, 0xFF 结尾 */
} CAN_ID_Entry_t;

static const CAN_ID_Entry_t s_whitelist[] = {
    /* 电机控制 ID（RX）*/
    { CAN_MOTOR_TURN_CMD_STDID,        0,    {0x11, 0x07, 0x08, 0x02, 0xFF} },
    { CAN_MOTOR_POWER_CMD_STDID,       1,    {0x11, 0x07, 0x08, 0x02, 0xFF} },

    /* 状态/日志 ID（RX）*/
    { CAN_MOTOR_TURN_CMD_STATUS_STDID,  0,    {0x01, 0x04, 0x05, 0xFF} },
    { CAN_MOTOR_POWER_CMD_STATUS_STDID, 1,    {0x01, 0x04, 0x05, 0xFF} },

    /* 全车命令 ID（RX）*/
    { CAN_CMD_STOP_STDID,               0xFF, {0x08, 0x11, 0xFF} },
    { CAN_CMD_TURN_STDID,               0,    {0x07, 0x08, 0x02, 0x11, 0xFF} },
    { CAN_CMD_POWER_STDID,              1,    {0x07, 0x08, 0x02, 0x11, 0xFF} },
};
#define WHITELIST_COUNT (sizeof(s_whitelist) / sizeof(s_whitelist[0]))

/* ===================== 静态辅助函数 ===================== */

static int find_entry(uint32_t stdId)
{
    for (uint32_t i = 0; i < WHITELIST_COUNT; i++) {
        if (s_whitelist[i].std_id == stdId)
            return (int)i;
    }
    return -1;
}

/* 命令字节 → CommandType_t, 某些命令需要结合 stdId 判断合法性 */
static CommandType_t map_cmd_byte_to_type(uint32_t stdId, uint8_t cmdByte)
{
    switch (cmdByte) {
        case 0x11: /* CAN_CMD_SET_SPEED_T2 */
        case 0x07: return CAN_CMD_SET_SPEED;
        case 0x08: return CAN_CMD_STOP;
        case 0x02: return CMD_REVERSE;
        case 0x01:
            if (stdId == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                stdId == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                return CMD_QUERY_STATUS;
            break;
        case 0x04:
            if (stdId == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                stdId == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                return CMD_LOG_START;
            break;
        case 0x05:
            if (stdId == CAN_MOTOR_TURN_CMD_STATUS_STDID ||
                stdId == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                return CMD_LOG_STOP;
            break;
        default:
            break;
    }
    return CMD_NONE;
}

/* ===================== 公共 API ===================== */

CAN_FilterResult_t CAN_Filter_Accept(uint32_t stdId, uint8_t cmdByte)
{
    int idx = find_entry(stdId);
    if (idx < 0)
        return CAN_FILTER_REJECT;

    const CAN_ID_Entry_t *entry = &s_whitelist[idx];
    for (int j = 0; entry->allowed_cmds[j] != 0xFF; j++) {
        if (entry->allowed_cmds[j] == cmdByte)
            return CAN_FILTER_ACCEPT;
    }
    return CAN_FILTER_REJECT;
}

uint8_t CAN_Filter_GetMotorId(uint32_t stdId)
{
    int idx = find_entry(stdId);
    return (idx >= 0) ? s_whitelist[idx].motor_id : 0;
}

CommandType_t CAN_Filter_CmdByteToType(uint32_t stdId, uint8_t cmdByte)
{
    return map_cmd_byte_to_type(stdId, cmdByte);
}

int16_t CAN_Filter_GetValue(uint32_t stdId, uint8_t cmdByte, const uint8_t rxData[8])
{
    (void)stdId;
    switch (cmdByte) {
        case 0x11: return (int8_t)rxData[1]; /* CAN_CMD_SET_SPEED_T2 */
        case 0x07: return (int8_t)rxData[CAN_DATA_INDEX_SPEED];
        default:   return 0;
    }
}
