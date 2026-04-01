#include "app_includes.h"
// #include "can.h"

extern osMutexId_t motor_mutexHandle;
extern CAN_HandleTypeDef hcan;

/* 判断是否为电机控制命令（需要投递到MotorQueue） */
static inline int is_motor_cmd(CommandType_t type)
{
    return type == CMD_FORWARD    || type == CMD_REVERSE  ||
           type == CMD_STOP       || type == CMD_SET_SPEED ||
           type == CAN_CMD_SET_SPEED || type == CAN_CMD_STOP;
}

void Command_Task(void *argument)
{
    CommandMsg_t cmd;

    for (;;)
    {
        if (osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever) != osOK)
            continue;

        // 过滤无效命令
        if (cmd.type == CMD_NONE)
            continue;

        // 读取当前电机状态（所有命令都可能需要）
        MotorStatus_t status = {0};
        if (osMutexAcquire(motor_mutexHandle, osWaitForever) == osOK)
        {
            status = g_motor_status;
            osMutexRelease(motor_mutexHandle);
        }

        /* ===== 数据流控制命令 ===== */
        if (cmd.type == CMD_LOG_START) { g_logger_enabled = 1; continue; }
        if (cmd.type == CMD_LOG_STOP)  { g_logger_enabled = 0; continue; }

        /* ===== 查询命令：直接处理，不进电机队列 ===== */
        if (cmd.type == CMD_LIST_STATUS || cmd.type == CMD_QUERY_STATUS)
        {
            if (cmd.type == CMD_QUERY_STATUS)
            {
                // CAN回复帧 0x301：[target(2B), current(2B), pwm(2B), reserved(2B)]
                CAN_TxHeaderTypeDef txHeader = {
                    .StdId = 0x323,
                    .DLC   = 8,
                    .IDE   = CAN_ID_STD,
                    .RTR   = CAN_RTR_DATA,
                };
                uint8_t txData[8] = {0};
                int16_t target  = (int16_t)status.target_logic_speed;
                int16_t current = (int16_t)status.current_logic_speed;
                int16_t pwm     = (int16_t)status.pwm_output;
                memcpy(&txData[0], &target,  2);
                memcpy(&txData[2], &current, 2);
                memcpy(&txData[4], &pwm,     2);
                uint32_t txMailbox;
                HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
            }

            // CMD_LIST_STATUS 的UART输出由Ack_Task处理，正常生成ACK即可
            AckMsg_t ack = {
                .type = cmd.type, .value = 0, .ok = 1,
                .current_logic_speed = status.current_logic_speed,
                .pwm_output          = status.pwm_output,
            };
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
            continue;
        }

        /* ===== 控制命令：生成ACK + 投递电机队列 ===== */
        AckMsg_t ack = {
            .type  = cmd.type,
            .value = is_motor_cmd(cmd.type) ? cmd.value : 0,
            .ok    = is_motor_cmd(cmd.type) ? 1 : 0,
            .current_logic_speed = status.current_logic_speed,
            .pwm_output          = status.pwm_output,
        };
        osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

        if (is_motor_cmd(cmd.type))
            osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
    }
}