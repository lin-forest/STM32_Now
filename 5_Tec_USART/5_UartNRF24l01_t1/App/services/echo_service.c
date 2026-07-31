#include "echo_service.h"
#include "app_config.h"
#include "stm32f1xx_hal.h"
#include "usart.h"

/* ===================== 私有变量 ===================== */

static uint8_t rx_byte;          // 中断接收单字节缓冲区
static uint8_t tx_byte;          // 中断发送单字节缓冲区

/* ===================== 实现 ===================== */

/**
 * @brief  启动 USART1 中断接收，进入回显模式
 */
void Echo_Init(void)
{
    // 以中断方式接收一个字节
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/**
 * @brief  UART RX 完成回调（HAL 弱函数重写）
 * @note   收到一个字节后立即以中断方式发回，并重新开启接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 保存收到的字节到发送缓冲区，避免 DMA 读取时被覆盖
        tx_byte = rx_byte;

        // 以中断方式发回（非阻塞，由 DMA 完成）
        HAL_UART_Transmit_IT(&huart1, &tx_byte, 1);

        // 立即重新开启中断接收（USART RX 与 TX 硬件独立，互不干扰）
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
