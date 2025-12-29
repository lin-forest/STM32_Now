
#include "stdarg.h" // For va_list, va_start, va_end
#include "app_includes.h"

/* ===========================================================
 * UART2 接收缓冲区（行缓冲模式）
 * =========================================================== */
static uint8_t uart2_rx_buf[LOGGER_UART_RX_BUF_LEN]; // 修正为 LOGGER_UART_RX_BUF_LEN
static uint16_t uart2_rx_index = 0;
static uint8_t  uart2_rx_byte;

/* ===========================================================
 * UART2 发送控制
 * =========================================================== */
static volatile uint8_t uart2_tx_busy = 0;
static uint8_t uart2_tx_buf[LOGGER_UART_TX_BUF_LEN]; // 使用 LOGGER_UART_TX_BUF_LEN

/* ===========================================================
 * Logger 应用层初始化
 * =========================================================== */
LoggerStatus_t Logger_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);   // 开始接收
    return LOGGER_OK;
}

/* ===========================================================
 * Logger 异步打印
 * =========================================================== */
LoggerStatus_t Logger_Print(const char *msg)
{
    if (uart2_tx_busy)
        return LOGGER_BUSY;     // 若上一次 TX 未完成 → 放弃本条（下位机常用策略）

    uint16_t len = strlen(msg);
    if (len >= sizeof(uart2_tx_buf))
        len = sizeof(uart2_tx_buf) - 1;

    memcpy(uart2_tx_buf, msg, len);
    uart2_tx_busy = 1;

    HAL_UART_Transmit_IT(&huart2, uart2_tx_buf, len); // 移除超时参数
    return LOGGER_OK;
}

/* ===========================================================
 * Logger 格式化打印
 * =========================================================== */
LoggerStatus_t Logger_Printf(const char *fmt, ...)
{
    char buffer[LOGGER_UART_TX_BUF_LEN]; // 临时缓冲区
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0 || len >= sizeof(buffer)) {
        // 错误或缓冲区溢出
        return LOGGER_ERROR;
    }

    return Logger_Print(buffer);
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
        if (uart2_rx_index < LOGGER_UART_RX_BUF_LEN - 1) // 修正为 LOGGER_UART_RX_BUF_LEN
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