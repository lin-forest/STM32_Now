#include "app_includes.h"

extern CAN_HandleTypeDef hcan;

/* 判断是否为电机控制命令（需要投递到MotorQueue） */
static inline int is_motor_cmd(CommandType_t type)
{
    return type == CMD_FORWARD    || type == CMD_REVERSE  ||
           type == CMD_STOP       || type == CMD_SET_SPEED ||
           type == CAN_CMD_SET_SPEED || type == CAN_CMD_STOP;
}

/* ── 发送单电机 CAN 状态帧（新格式）──
 *   [0-1] current_logic_speed    (int16)
 *   [2-3] accumulated_ticks      (uint16, low 16 bits)
 *   [4-5] pwm_output             (int16)
 *   [6]   target_logic_speed     (int8,  -100..100)
 *   [7]   flags                  (uint8, MOTOR_FLAG_*)
 */
static void send_motor_status(uint8_t mid)
{
    osMutexId_t myMutex = (mid == 0) ? motor0_mutexHandle : motor1_mutexHandle;
    uint32_t replyId = (mid == 0) ? CAN_MOTOR_TURN_STATUS_STDID
                                   : CAN_MOTOR_POWER_STATUS_STDID;

    CAN_TxHeaderTypeDef txHeader = {
        .StdId = replyId,
        .DLC   = 8,
        .IDE   = CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
    };
    uint8_t txData[8] = {0};

    if (osMutexAcquire(myMutex, osWaitForever) == osOK)
    {
        int16_t  current = (int16_t)             g_motors[mid].current_logic_speed;
        uint16_t accum   = (uint16_t)(            g_motors[mid].accumulated_ticks & 0xFFFF);
        int16_t  pwm     =                        g_motors[mid].pwm_output;
        int8_t   target  = (int8_t)               g_motors[mid].target_logic_speed;
        uint8_t  flags   =                        g_motors[mid].flags;

        memcpy(&txData[0], &current, 2);
        memcpy(&txData[2], &accum,   2);
        memcpy(&txData[4], &pwm,     2);
        txData[6] = (uint8_t)target;
        txData[7] = flags;

        osMutexRelease(myMutex);
    }

    uint32_t txMailbox;
    HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

void Command_Task(void *argument)
{
    CommandMsg_t cmd;
    uint32_t last_status_tick = 0;

    for (;;)
    {
        /* ── 接收命令（50ms 超时，为主动上报让路）── */
        osStatus_t q_status = osMessageQueueGet(CommandQueueHandle, &cmd, NULL, 50);

        if (q_status == osOK && cmd.type != CMD_NONE)
        {
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
                    send_motor_status(mid);     // 新格式响应

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

        /* ===== 主动状态上报（50ms 心跳, ~20Hz） ===== */
        uint32_t now = HAL_GetTick();
        if (now - last_status_tick >= 50)
        {
            send_motor_status(0);
            send_motor_status(1);
            last_status_tick = now;
        }
    }
}