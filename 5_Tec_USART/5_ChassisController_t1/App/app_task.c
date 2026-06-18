#include "app_includes.h"
#include "usart.h"
#include <string.h>

/**
 * @brief 引用在usart.c中定义的环形缓冲区
 */
extern RingBuffer_t uart1_rx_buffer;

/**
 * @brief UART1 DMA 发送静态缓冲区
 *
 * HAL_UART_Transmit_DMA 要求缓冲区在 DMA 传输期间持续有效（不能是栈变量），
 * 因此使用静态数组持有数据。互斥锁保证同一时刻只有一路发送在进行，
 * 信号量由 HAL_UART_TxCpltCallback 在传输结束后释放，解除任务等待。
 */
#define UART1_TX_DMA_BUF_SIZE  128u
static uint8_t uart1_tx_dma_buf[UART1_TX_DMA_BUF_SIZE];

/**
 * @brief uartToCanQueue 丢帧计数器
 *
 * ProtocolParser 入队返回非 osOK（队列满）时递增。
 */
static uint32_t uartToCanQueue_drop_cnt = 0;

/**
 * @brief canTxQueue 丢帧计数器
 *
 * CommandProcess 入队返回非 osOK（队列满）时递增。
 */
static uint32_t canTxQueue_drop_cnt = 0;

/**
 * @brief UART1 DMA 发送信号量超时 (毫秒)
 *
 * 防止 DMA 启动失败或中断丢失导致 osSemaphoreAcquire 永久阻塞。
 * 1000ms 远大于 UART 115200bps 下 128 字节的传输时间 (~11ms)。
 */
#define UART1_TX_SEM_TIMEOUT_MS  1000u

/* ===================== 全局系统状态实例定义 ===================== */
System_State_t g_system_state = {0};

/**
 * @brief 带互斥锁的 UART1 DMA 发送封装
 *
 * 流程：
 *   1. Mutex Acquire  —— 独占发送通道
 *   2. 拷贝数据到静态缓冲区
 *   3. HAL_UART_Transmit_DMA —— 启动传输，立即返回
 *   4. Semaphore Acquire  —— 挂起等待 TxCplt 回调（带超时防死锁）
 *   5. Mutex Release  —— 释放通道
 *
 * @note  DMA 启动失败或信号量超时时自动释放 mutex，避免死锁。
 */
static void uart1_send(const char *buf, uint16_t len)
{
    if (len == 0 || len > UART1_TX_DMA_BUF_SIZE) { return; }

    osMutexAcquire(uart1_tx_mutexHandle, osWaitForever);

    memcpy(uart1_tx_dma_buf, buf, len);

    /* DMA 启动失败 → 不会触发 TxCpltCallback → 直接释放 mutex 返回 */
    if (HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma_buf, len) != HAL_OK)
    {
        osMutexRelease(uart1_tx_mutexHandle);
        return;
    }

    /* 等待 TxCplt 信号量，超时说明中断丢失，释放 mutex 防止死锁 */
    if (osSemaphoreAcquire(uart1_tx_semHandle, UART1_TX_SEM_TIMEOUT_MS) != osOK)
    {
        osMutexRelease(uart1_tx_mutexHandle);
        return;
    }

    osMutexRelease(uart1_tx_mutexHandle);
}

/* ===================== 辅助：根据 CAN ID 获取电机索引 ===================== */
/**
 * @brief  将 CAN 状态反馈 ID 映射到电机索引
 * @param  stdId: CAN 标准 ID
 * @return 电机索引 (0=MOTOR_TURN, 1=MOTOR_POWER)，未知 ID 返回 0xFF
 */
static uint8_t can_id_to_motor_idx(uint32_t stdId)
{
    if (stdId == CAN_MOTOR_TURN_STATUS_STDID)   return 0;
    if (stdId == CAN_MOTOR_POWER_STATUS_STDID)  return 1;
    return 0xFF;  // 未知
}

/* ===================== 辅助：解码 MCLM_t2 状态帧 ===================== */
/**
 * @brief  解码 MCLM_t2 发出的 8 字节电机状态帧
 *
 * 帧格式（与 MCLM_t2 command_task.c 的 send_motor_status 一致）：
 *   [0-1] current_logic_speed  (int16, LE)
 *   [2-3] accumulated_ticks    (uint16, LE)
 *   [4-5] pwm_output           (int16, LE)
 *   [6]   target_logic_speed   (int8)
 *   [7]   flags                (uint8)
 */
static void decode_motor_status(const uint8_t data[8], Motor_State_t *out)
{
    int16_t  speed;
    int16_t  pwm;
    memcpy(&speed, &data[CAN_STATUS_IDX_CURRENT_SPEED], 2);
    memcpy(&pwm,   &data[CAN_STATUS_IDX_PWM_OUTPUT],    2);

    out->current_speed = speed;
    out->pwm_output    = pwm;
    out->target_speed  = (int8_t)data[CAN_STATUS_IDX_TARGET_SPEED];
    out->flags         = data[CAN_STATUS_IDX_FLAGS];
    out->last_update_tick = HAL_GetTick();
}

/**
 * @brief  UART到CAN转换任务的实现函数 (Phase 1 简化版：纯 CAN 发送器)
 *
 * 从 canTxQueue 获取待发送的 CAN 帧，调用 HAL 发送。
 * 不再直接消费 uartToCanQueue，该队列由 CommandProcess_Task 消费。
 *
 * @param  argument: 未使用
 */
void UartToCan_Task_Run(void *argument)
{
    App_CAN_Message_t can_msg;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    char dbg_buffer[128];

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;

    for(;;)
    {
        if (osMessageQueueGet(canTxQueueHandle, &can_msg, NULL, osWaitForever) == osOK)
        {
            /* 根据 ID 判断标准帧 / 扩展帧 */
            if (can_msg.id > 0x7FF) {
                tx_header.IDE = CAN_ID_EXT;
                tx_header.ExtId = can_msg.id;
            } else {
                tx_header.IDE = CAN_ID_STD;
                tx_header.StdId = (uint16_t)can_msg.id;
            }

            tx_header.DLC = can_msg.len;

            HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header, can_msg.data, &tx_mailbox);

            int offset = sprintf(dbg_buffer, "CAN_TX OK | ID=0x%lX DLC=%d Done=%lu\r\n",
                             can_msg.id, can_msg.len, can_tx_done_cnt);
            uart1_send(dbg_buffer, offset);
        }
    }
}

/**
 * @brief  CAN接收处理任务的实现函数 (Phase 1 重构：状态更新 + 调试打印)
 *
 *   - 电机状态帧 (0x323/0x324)：解码并更新 g_system_state.motor[]
 *   - 其他 CAN 帧：格式化为可读字符串后 UART 输出（保留原有调试能力）
 *
 * @param  argument: 未使用
 */
void CanRxProcess_Task_Run(void *argument)
{
    App_CAN_Message_t rx_can_msg;
    // char tx_buffer[128];  // [CAN→UART 打印] 暂取消：MCLM持续状态帧淹没UART，需用时取消注释

    for(;;)
    {
        if (osMessageQueueGet(canRxQueueHandle, &rx_can_msg, NULL, osWaitForever) == osOK)
        {
            // static uint32_t last_state_print = 0;  // [CAN→UART 限频] 暂取消
            // uint32_t now = HAL_GetTick();           // [CAN→UART 限频] 暂取消
            uint8_t midx = can_id_to_motor_idx(rx_can_msg.id);

            if (midx != 0xFF)
            {
                /* ── 已知状态帧：解码并更新 SystemState（核心功能，保留）── */
                if (rx_can_msg.len >= 8) {
                    decode_motor_status(rx_can_msg.data, &g_system_state.motor[midx]);
                }

                /* ===== [CAN→UART 打印] 暂取消 =====
                 * 原因：MCLM 每 50ms 持续发状态帧 → 不停调用 uart1_send
                 *      → UART TX 互斥锁激烈竞争 → 阻塞 CommandProcess/UartToCan 的诊断打印
                 * 恢复：取消下面这段注释即可
                 * 限频：恢复后每 950ms 打印一次，不会淹没
                 */
                // if (now - last_state_print >= 950)
                // {
                //     last_state_print = now;
                //     int offset = sprintf(tx_buffer, "STATE | Motor%d speed=%d pwm=%d flags=0x%02X\r\n",
                //                          midx,
                //                          g_system_state.motor[midx].current_speed,
                //                          g_system_state.motor[midx].pwm_output,
                //                          g_system_state.motor[midx].flags);
                //     uart1_send(tx_buffer, offset);
                // }
            }
            // else
            // {
            //     /* ===== [CAN→UART 打印] 未知帧，暂取消 ===== */
            //     if (now - last_state_print >= 950)
            //     {
            //         last_state_print = now;
            //         int offset = sprintf(tx_buffer, "CAN RX | ID: 0x%03lX | DLC: %d | Data: ",
            //                              rx_can_msg.id, rx_can_msg.len);
            //         for (int i = 0; i < rx_can_msg.len && i < 8; i++)
            //         {
            //             offset += sprintf(tx_buffer + offset, "%02X ", rx_can_msg.data[i]);
            //         }
            //         offset += sprintf(tx_buffer + offset, "\r\n");
            //         uart1_send(tx_buffer, offset);
            //     }
            // }
        }
    }
}

/**
 * @brief  心跳任务的实现函数 (不变)
 */
void Heartbeat_Task_Run(void *argument)
{
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(300);
    }
}

/**
 * @brief  协议解析任务的实现函数 (不变)
 *
 * 状态机：STATE_WAIT_SOF → STATE_WAIT_CMD → STATE_WAIT_ID → STATE_WAIT_LEN → STATE_WAIT_DATA
 * 解析结果放入 uartToCanQueue，由 CommandProcess_Task 消费。
 */
void ProtocolParser_Task_Run(void *argument)
{
    ParserState_t state = STATE_WAIT_SOF;
    App_UART_Message_t current_msg;
    uint8_t byte_received;
    // uint32_t frame_count = 0;   // [Phase1诊断] 统计解析帧数，用完后注释。若启用需同时取消下方 DBG 打印注释
    uint8_t data_idx = 0;
    uint8_t id_byte_count = 0;

    for(;;)
    {
        if (ring_buffer_get(&uart1_rx_buffer, &byte_received))
        {
            switch (state)
            {
                case STATE_WAIT_SOF:
                    if (byte_received == FRAME_SOF) {
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        id_byte_count = 0;
                        data_idx = 0;
                        state = STATE_WAIT_CMD;
                    }
                    break;

                case STATE_WAIT_CMD:
                    if (byte_received == FRAME_SOF) {
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        id_byte_count = 0;
                        data_idx = 0;
                        // SOF 已消耗，下字节是 cmd
                    } else {
                        current_msg.cmd = byte_received;
                        state = STATE_WAIT_ID;
                    }
                    break;

                case STATE_WAIT_ID:
                    if (byte_received == FRAME_SOF) {
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        id_byte_count = 0;
                        data_idx = 0;
                        state = STATE_WAIT_CMD;
                    } else {
                        current_msg.id |= (uint32_t)byte_received << (8 * id_byte_count);
                        id_byte_count++;
                        if (id_byte_count >= 4) {
                            state = STATE_WAIT_LEN;
                        }
                    }
                    break;

                case STATE_WAIT_LEN:
                    if (byte_received == FRAME_SOF) {
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        id_byte_count = 0;
                        data_idx = 0;
                        state = STATE_WAIT_CMD;
                    } else if (byte_received <= 8) {
                        current_msg.len = byte_received;
                        data_idx = 0;
                        if (current_msg.len > 0) {
                            state = STATE_WAIT_DATA;
                        } else {
                            // frame_count++;  // [Phase1诊断] 统计解析帧数
                            // uart1_send("DBG: PP frame parsed\r\n", 22);  // [Phase1诊断] 有UART竞争问题，验证完后注释
                            if (osMessageQueuePut(uartToCanQueueHandle, &current_msg, 0, 0) != osOK) {
                                uartToCanQueue_drop_cnt++;
                            }
                            state = STATE_WAIT_SOF;
                        }
                    } else {
                        state = STATE_WAIT_SOF;
                    }
                    break;

                case STATE_WAIT_DATA:
                    if (byte_received == FRAME_SOF) {
                        memset(&current_msg, 0, sizeof(App_UART_Message_t));
                        id_byte_count = 0;
                        data_idx = 0;
                        state = STATE_WAIT_CMD;
                    } else {
                        current_msg.data[data_idx++] = byte_received;
                        if (data_idx >= current_msg.len) {
                            // frame_count++;  // [Phase1诊断] 统计解析帧数
                            // uart1_send("DBG: PP frame parsed (data)\r\n", 29);  // [Phase1诊断]
                            if (osMessageQueuePut(uartToCanQueueHandle, &current_msg, 0, 0) != osOK) {
                                uartToCanQueue_drop_cnt++;
                            }
                            state = STATE_WAIT_SOF;
                        }
                    }
                    break;
            }
        }
        else
        {
            /* 缓冲区为空，等待 ISR 通知 */
            osEventFlagsWait(uart1_rx_eventHandle, UART1_RX_FLAG, osFlagsWaitAny, 10);
        }
    }
}

/**
 * @brief  命令处理任务的实现函数 (Phase 1 新增)
 *
 * 从 uartToCanQueue 获取解析后的 UART 帧，根据 cmd 字段：
 *   - CMD_SET_SPEED (0x01)：更新 SystemState target_speed，转发 CAN
 *   - CMD_ESTOP (0x04)：置位 estop 标志，发送全车停止 CAN 帧
 *   - CMD_GET_STATE (0x02)：从 SystemState 读取状态，UART 回传
 *   - 其他：透传至 CAN（保留网关兼容性）
 *
 * @param  argument: 未使用
 */
void CommandProcess_Task_Run(void *argument)
{
    App_UART_Message_t uart_msg;
    App_CAN_Message_t  can_tx;
    char dbg_buffer[128];

    /* [Phase1诊断] 任务启动确认。有UART竞争问题，验证完后注释 */
    // uart1_send("DBG: CommandProcess_Task started\r\n", 34);

    for(;;)
    {
        if (osMessageQueueGet(uartToCanQueueHandle, &uart_msg, NULL, osWaitForever) == osOK)
        {
            /* [Phase1诊断] 收到消息确认。有UART竞争问题，验证完后注释 */
            // uart1_send("DBG: CMD received\r\n", 19);

            /* 更新系统运行时间 */
            g_system_state.flag.uptime_ms = HAL_GetTick();

            switch (uart_msg.cmd)
            {
                case CMD_SET_SPEED:
                {
                    /* 映射到电机索引 */
                    uint8_t midx = 0xFF;
                    if (uart_msg.id == CAN_MOTOR_TURN_CMD_STDID)       midx = 0;
                    else if (uart_msg.id == CAN_MOTOR_POWER_CMD_STDID) midx = 1;
                    /* 也支持全车命令 ID */
                    else if (uart_msg.id == CAN_CMD_TURN_STDID)        midx = 0;
                    else if (uart_msg.id == CAN_CMD_POWER_STDID)       midx = 1;

                    if (midx < MOTOR_MAX_COUNT) {
                        /* 从 data[1] 提取速度值（与 MCLM_t2 协议兼容） */
                        int16_t speed = (int8_t)uart_msg.data[1];
                        g_system_state.motor[midx].target_speed = speed;
                    }

                    /* 构建 CAN 帧：替换 data[0] 为 CAN 命令字节 0x11 */
                    can_tx.id   = uart_msg.id;
                    can_tx.len  = uart_msg.len;
                    memcpy(can_tx.data, uart_msg.data, uart_msg.len);
                    can_tx.data[0] = CAN_CMD_SET_SPEED_T2;  // 0x11

                    if (osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0) != osOK) {
                        canTxQueue_drop_cnt++;
                    }
                    break;
                }

                case CMD_ESTOP:
                {
                    g_system_state.flag.estop = 1;

                    /* 发送全车停止帧 (CAN ID 0x101, data[0]=0x08) */
                    can_tx.id   = CAN_CMD_STOP_STDID;  // 0x101
                    can_tx.len  = 1;
                    can_tx.data[0] = 0x08;  // CAN_CMD_STOP

                    if (osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0) != osOK) {
                        canTxQueue_drop_cnt++;
                    }

                    /* 也发送到各电机控制 ID */
                    can_tx.id   = CAN_MOTOR_TURN_CMD_STDID;
                    can_tx.len  = 2;
                    can_tx.data[0] = 0x08;
                    can_tx.data[1] = 0;
                    osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0);

                    can_tx.id   = CAN_MOTOR_POWER_CMD_STDID;
                    osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0);

                    int offset = sprintf(dbg_buffer, "ESTOP TRIGGERED\r\n");
                    uart1_send(dbg_buffer, offset);
                    break;
                }

                case CMD_GET_STATE:
                {
                    /* 从 SystemState 读取并回传 */
                    uint8_t midx = 0;
                    if (uart_msg.id == CAN_MOTOR_POWER_CMD_STDID ||
                        uart_msg.id == CAN_MOTOR_POWER_CMD_STATUS_STDID)
                        midx = 1;

                    int offset = sprintf(dbg_buffer,
                        "STATE_RSP | Motor%d current=%d target=%d pwm=%d flags=0x%02X "
                        "estop=%d uptime=%lu\r\n",
                        midx,
                        g_system_state.motor[midx].current_speed,
                        g_system_state.motor[midx].target_speed,
                        g_system_state.motor[midx].pwm_output,
                        g_system_state.motor[midx].flags,
                        g_system_state.flag.estop,
                        g_system_state.flag.uptime_ms);
                    uart1_send(dbg_buffer, offset);
                    break;
                }

                case CMD_SET_MODE:
                {
                    g_system_state.flag.mode = uart_msg.data[0];

                    /* 透传到 CAN（保留网关能力） */
                    can_tx.id   = uart_msg.id;
                    can_tx.len  = uart_msg.len;
                    memcpy(can_tx.data, uart_msg.data, uart_msg.len);

                    if (osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0) != osOK) {
                        canTxQueue_drop_cnt++;
                    }
                    break;
                }

                default:
                {
                    /* 其他命令：保持透传（不修改命令字节） */
                    can_tx.id   = uart_msg.id;
                    can_tx.len  = uart_msg.len;
                    memcpy(can_tx.data, uart_msg.data, uart_msg.len);

                    if (osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0) != osOK) {
                        canTxQueue_drop_cnt++;
                    }
                    break;
                }
            }
        }
    }
}
