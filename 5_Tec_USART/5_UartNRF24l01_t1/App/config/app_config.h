#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== USART Echo 配置 ===================== */
#define ECHO_UART_HANDLE        huart1       // 使用 USART1
#define ECHO_BAUDRATE           115200       // 波特率
#define ECHO_RX_BUF_SIZE        64           // RX 环形缓冲区大小
#define ECHO_TX_BUF_SIZE        64           // TX 环形缓冲区大小

/* ===================== LED 指示灯 ===================== */
#define LED_TOGGLE_DELAY_MS     500          // LED 闪烁间隔(ms)

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */
