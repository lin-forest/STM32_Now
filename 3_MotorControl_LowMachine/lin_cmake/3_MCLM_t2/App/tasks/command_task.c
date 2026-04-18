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

        // 1. 确定目标电机索引 (安全性检查)
        uint8_t idx = (cmd.motor_index < MOTOR_COUNT) ? cmd.motor_index : 0;

        // 2. 读取目标电机状态（短超时，非关键路径）
        Motor_t status = {0};
        if (osMutexAcquire(motor_mutexHandle, 5) == osOK)
        {
            status = g_motors[idx];
            osMutexRelease(motor_mutexHandle);
        }

        /* ===== 数据流控制命令 ===== */
        if (cmd.type == CMD_LOG_START || cmd.type == CMD_LOG_STOP)
        {
            g_logger_enabled = (cmd.type == CMD_LOG_START);
            
            // 根据电机索引选择对应的反馈 ID (0x225 或 0x235)
            uint32_t resp_id = (idx == 0) ? CAN_MOTOR1_CMD_STATUS_STDID : CAN_MOTOR2_CMD_STATUS_STDID;

            // 1. 发送 CAN 反馈
            // 让 CAN 总线上的发起者知道日志状态已切换
            CAN_TxHeaderTypeDef txHeader = {
                .StdId = resp_id,
                .DLC   = 8,
                .IDE   = CAN_ID_STD,
                .RTR   = CAN_RTR_DATA,
            };
            uint8_t txData[8] = {0};
            txData[0] = 0xCF; // Command Feedback 标识码
            txData[1] = (uint8_t)cmd.type;
            txData[2] = (uint8_t)g_logger_enabled;
            txData[7] = idx;  // 在反馈帧中带上电机索引
            
            uint32_t txMailbox;
            HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

            // 2. 生成 UART ACK 消息，让 Ack_Task 进行打印反馈
            AckMsg_t ack = {
                .type  = cmd.type,
                .value = g_logger_enabled,
                .ok    = 1,
                .measured_speed = (int16_t)status.measured_speed,
                .pwm_output          = (int16_t)status.pwm_output,
            };
            osMessageQueuePut(AckQueueHandle, &ack, 0, 0);
            
            continue; 
        }

        /* ===== 查询命令：直接处理，不进电机队列 ===== */
        if (cmd.type == CMD_LIST_STATUS || cmd.type == CMD_QUERY_STATUS)
        {
            if (cmd.type == CMD_QUERY_STATUS)
            {
                // 根据电机索引选择对应的发送 ID (0x325 或 0x335)
                uint32_t status_id = (idx == 0) ? CAN_MOTOR1_STATUS_STDID : CAN_MOTOR2_STATUS_STDID;

                // CAN回复帧：[target(2B), current(2B), pwm(2B), reserved(2B)]
                CAN_TxHeaderTypeDef txHeader = {
                    .StdId = status_id,
                    .DLC   = 8,
                    .IDE   = CAN_ID_STD,
                    .RTR   = CAN_RTR_DATA,
                };
                uint8_t txData[8] = {0};
                int16_t target  = (int16_t)status.target_logic_speed;
                int16_t current = (int16_t)status.measured_speed;
                int16_t pwm     = (int16_t)status.pwm_output;
                memcpy(&txData[0], &target,  2);
                memcpy(&txData[2], &current, 2);
                memcpy(&txData[4], &pwm,     2);
                txData[7] = idx; // 标识该数据属于哪个电机
                uint32_t txMailbox;
                HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
            }

            // CMD_LIST_STATUS 的UART输出由Ack_Task处理，正常生成ACK即可
            AckMsg_t ack = {
                .type = cmd.type, .value = 0, .ok = 1,
                .measured_speed = status.measured_speed,
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
            .measured_speed      = status.measured_speed, // 直接反馈测量速度，便于调试观察
            .pwm_output          = status.pwm_output,
        };
        osMessageQueuePut(AckQueueHandle, &ack, 0, 0);

        if (is_motor_cmd(cmd.type))
            osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
    }
}