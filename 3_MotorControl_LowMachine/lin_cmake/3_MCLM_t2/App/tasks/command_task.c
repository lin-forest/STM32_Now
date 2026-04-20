#include "app_includes.h"

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

        // 读取目标电机状态
        uint8_t mid = (cmd.motor_id < MOTOR_COUNT) ? cmd.motor_id : 0;
        osMutexId_t myMutex = (mid == 0) ? motor0_mutexHandle : motor1_mutexHandle;

        Motor_t status = {0};
        if (osMutexAcquire(myMutex, osWaitForever) == osOK)
        {
            status = g_motors[mid];
            osMutexRelease(myMutex);
        }

        /* ===== 数据流控制命令 ===== */
        if (cmd.type == CMD_LOG_START || cmd.type == CMD_LOG_STOP)
        {
            g_logger_enabled = (cmd.type == CMD_LOG_START);

            CAN_TxHeaderTypeDef txHeader = {
                .StdId = CAN_MOTOR_TURN_CMD_STATUS_STDID,
                .DLC   = 8,
                .IDE   = CAN_ID_STD,
                .RTR   = CAN_RTR_DATA,
            };
            uint8_t txData[8] = {0};
            txData[0] = 0xCF;
            txData[1] = (uint8_t)cmd.type;
            txData[2] = (uint8_t)g_logger_enabled;

            uint32_t txMailbox;
            HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

            AckMsg_t ack = {
                .type  = cmd.type,
                .value = g_logger_enabled,
                .ok    = 1,
                .current_logic_speed = (int16_t)status.current_logic_speed,
                .pwm_output          = (int16_t)status.pwm_output,
            };
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
            continue;
        }

        /* ===== 查询命令 ===== */
        if (cmd.type == CMD_LIST_STATUS || cmd.type == CMD_QUERY_STATUS)
        {
            if (cmd.type == CMD_QUERY_STATUS)
            {
                // 按 motor_id 选回复 CAN ID
                uint32_t replyId = (mid == 0) ? CAN_MOTOR_TURN_STATUS_STDID
                                              : CAN_MOTOR_POWER_STATUS_STDID;
                CAN_TxHeaderTypeDef txHeader = {
                    .StdId = replyId,
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

            AckMsg_t ack = {
                .type = cmd.type, .value = 0, .ok = 1,
                .current_logic_speed = status.current_logic_speed,
                .pwm_output          = status.pwm_output,
            };
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
            continue;
        }

        /* ===== 控制命令：生成ACK + 按 motor_id 路由到专属队列 ===== */
        AckMsg_t ack = {
            .type  = cmd.type,
            .value = is_motor_cmd(cmd.type) ? cmd.value : 0,
            .ok    = is_motor_cmd(cmd.type) ? 1 : 0,
            .current_logic_speed = status.current_logic_speed,
            .pwm_output          = status.pwm_output,
        };
        osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

        if (is_motor_cmd(cmd.type))
        {
            if (cmd.motor_id == 0xFF)
            {
                // 广播：同时投递两个队列
                osMessageQueuePut(MotorQueueHandle,  &cmd, 0, 0);
                osMessageQueuePut(MotorQueue1Handle, &cmd, 0, 0);
            }
            else
            {
                osMessageQueueId_t q = (cmd.motor_id == 1) ? MotorQueue1Handle : MotorQueueHandle;
                osMessageQueuePut(q, &cmd, 0, 0);
            }
        }
    }
}