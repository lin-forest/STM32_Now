#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> // 包含此文件以使用 uint32_t, uint8_t 等类型

/* Public define -------------------------------------------------------------*/

// 定义队列中可以存储的最大CAN报文数量
#define CAN_RX_QUEUE_SIZE    16
#define UART_TO_CAN_QUEUE_SIZE 16


/* Public typedef ------------------------------------------------------------*/

/**
 * @brief 应用程序内部统一的CAN消息结构体
 * @note  这个结构体将用于在FreeRTOS队列中传递CAN报文
 */
typedef struct {
    uint32_t can_id;  // CAN ID (标准ID或扩展ID)
    uint8_t  dlc;     // 数据长度 (0-8)
    uint8_t  data[8]; // 数据负载
} App_CAN_Message_t;


/* Public macro --------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/


#endif /* APP_CONFIG_H */
