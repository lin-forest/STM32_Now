#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> // 包含此文件以使用 uint32_t, uint8_t 等类型

/* Public define -------------------------------------------------------------*/

// 定义队列中可以存储的最大CAN报文数量
#define CAN_RX_QUEUE_SIZE    16
#define UART_TO_CAN_QUEUE_SIZE 16


/* Public typedef ------------------------------------------------------------*/

// 协议帧头定义
#define FRAME_SOF 0xAA

/**
 * @brief 统一指令集枚举
 */
typedef enum {
    CMD_SET_SPEED = 0x01, // 设置速度
    CMD_GET_STATE = 0x02, // 获取状态
    CMD_SET_MODE  = 0x03, // 设置模式
    CMD_ESTOP     = 0x04, // 紧急停止
} Command_ID_t;

/**
 * @brief 串口消息结构体 (用于uartToCanQueue)
 * @note  这是从串口接收并解析后的结构化数据
 * @note  字段按对齐顺序排列: uint32_t优先, 避免3字节填充, sizeof=16字节
 * @note  cmd 字段是 UART 帧指令（Command_ID_t），仅用于网关内部路由/日志。
 *        CAN 命令字节请放入 data[0]（参考 CAN_CMD_* 宏定义），两者值不同。
 *        历史备注：早期注释写"cmd 透传至 CAN 总线"，但实际验证发现
 *        UART cmd 值与 CAN 命令码不兼容，强行透传会导致协议解析错误，
 *        因此保持 data[] 直通 CAN 的设计。
 */
typedef struct {
    uint32_t id;      // CAN ID (最大29位扩展帧)
    uint8_t  cmd;     // UART 帧指令 (Command_ID_t)，仅网关内部路由，不发送到 CAN
                      // CAN 命令码请用 CAN_CMD_* 宏填写到 data[0]
    uint8_t  len;     // 数据长度
    uint8_t  data[8]; // 数据负载 (最多8字节, 对齐CAN); data[0] 为 CAN 命令字节
} App_UART_Message_t;


/**
 * @brief 应用程序内部统一的CAN消息结构体
 * @note  这个结构体将用于在FreeRTOS队列中传递CAN报文
 */
typedef struct {
    uint32_t id;      // CAN ID (标准ID或扩展ID)
    uint8_t  len;     // 数据长度 (0-8)
    uint8_t  data[8]; // 数据负载
} App_CAN_Message_t;


/* Public macro --------------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/

/**
 * @brief 协议解析状态机状态枚举 (供 ProtocolParser_Task_Run 使用)
 */
typedef enum {
    STATE_WAIT_SOF,  // 等待帧头 (Start of Frame)
    STATE_WAIT_CMD,  // 等待指令ID
    STATE_WAIT_ID,   // 等待4字节的CAN ID
    STATE_WAIT_LEN,  // 等待数据长度
    STATE_WAIT_DATA  // 等待数据负载
} ParserState_t;


#endif /* APP_CONFIG_H */