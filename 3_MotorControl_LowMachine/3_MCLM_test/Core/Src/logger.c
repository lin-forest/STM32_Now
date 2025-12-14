

#include "logger.h"
#include "usart.h"
#include "string.h"
#include "command.h"
#include "app_task.h"     // 需要 CommandQueueHandle

/* ===========================================================
 * UART2 接收缓冲区（行缓冲模式）
 * =========================================================== */
static uint8_t uart2_rx_buf[UART2_RX_BUF_LEN];
static uint16_t uart2_rx_index = 0;
static uint8_t  uart2_rx_byte;

/* ===========================================================
 * UART2 发送控制
 * =========================================================== */
static volatile uint8_t uart2_tx_busy = 0;
static uint8_t uart2_tx_buf[64];

/* ===========================================================
 * UART2 应用层初始化
 * =========================================================== */
void UART_App_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);   // 开始接收
}

/* ===========================================================
 * UART2 异步打印
 * =========================================================== */
void UART2_Print(const char *msg)
{
    if (uart2_tx_busy)
        return;     // 若上一次 TX 未完成 → 放弃本条（下位机常用策略）

    uint16_t len = strlen(msg);
    if (len >= sizeof(uart2_tx_buf))
        len = sizeof(uart2_tx_buf) - 1;

    memcpy(uart2_tx_buf, msg, len);
    uart2_tx_busy = 1;

    HAL_UART_Transmit_IT(&huart2, uart2_tx_buf, len);
}

/* ===========================================================
 * UART2 接收中断
 * =========================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;

    uint8_t ch = uart2_rx_byte;

    /* ---------- 行结束符 ---------- */
    if (ch == '\r' || ch == '\n')
    {
        if (uart2_rx_index > 0)       // 避免空包
        {
            uart2_rx_buf[uart2_rx_index] = '\0';

            /* 解析命令 */
            CommandMsg_t msg = Command_ParseString((char*)uart2_rx_buf);

            /* 投递到队列，不等待 */
            osMessageQueuePut(CommandQueueHandle, &msg, 0, 0);

            // // 反馈设计
            // Command_Feedback(&msg);
        }

        uart2_rx_index = 0;          // 清空接收
    }

    /* ---------- 普通字符 ---------- */
    else
    {
        if (uart2_rx_index < UART2_RX_BUF_LEN - 1)
        {
            uart2_rx_buf[uart2_rx_index++] = ch;
        }
        else
        {
            uart2_rx_index = 0;       // 溢出保护
        }
    }

    /* 继续接收 */
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

/* ===========================================================
 * UART2 发送完成回调
 * =========================================================== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        uart2_tx_busy = 0;   // 恢复 TX 可用
}
