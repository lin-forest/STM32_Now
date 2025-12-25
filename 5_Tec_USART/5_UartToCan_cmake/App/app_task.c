#include "app_task.h"
#include "string.h" // For strcmp, strlen
#include "stdio.h"  // For sscanf

/* Private defines -----------------------------------------------------------*/
#define UART_RX_BUFFER_SIZE 64

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan; // Defined in can.c

/* Private function prototypes -----------------------------------------------*/
static CanCmdType_t GetCanCmdTypeFromString(const char *cmd_type_str);

/**
  * @brief  Initializes application tasks and queues.
  * @param  None
  * @retval None
  */
void App_Tasks_Init(void)
{
    // Queues are created in freertos.c, just ensure they are accessible via extern in main.h
    // Tasks are created in freertos.c, just ensure they are accessible via extern in main.h
}

/**
  * @brief  UART接收任务。
  *         从UART接收字符，构建命令字符串，并解析它们。
  * @param  argument: 未使用
  * @retval None
  */
void UartRxTask(void *argument)
{
    uint8_t uart_rx_byte; // 用于存储从UART接收到的单个字节
    static char rx_buffer[UART_RX_BUFFER_SIZE]; // 静态缓冲区，用于存储接收到的命令字符串
    static uint16_t rx_buffer_idx = 0; // 缓冲区当前写入位置的索引

    for (;;) // 任务的无限循环
    {
        // 尝试从UartRxQueueHandle队列中获取一个字节。
        // osWaitForever 表示如果队列为空，任务将一直等待直到有数据。
        if (osMessageQueueGet(UartRxQueueHandle, &uart_rx_byte, NULL, osWaitForever) == osOK)
        {
            // 检查接收到的字节是否是换行符或回车符，这通常标志着一个命令的结束
            if (uart_rx_byte == '\n' || uart_rx_byte == '\r')
            {
                // 如果缓冲区中有数据（即接收到了有效的命令）
                if (rx_buffer_idx > 0)
                {
                    rx_buffer[rx_buffer_idx] = '\0'; // 在命令字符串末尾添加空字符，使其成为一个C字符串
                    UartRxParser(rx_buffer); // 调用解析器函数处理这个完整的命令
                    rx_buffer_idx = 0; // 重置缓冲区索引，准备接收下一个命令
                }
            }
            else // 如果接收到的不是换行符，则将其添加到缓冲区
            {
                // 检查缓冲区是否还有空间，防止溢出
                if (rx_buffer_idx < (UART_RX_BUFFER_SIZE - 1))
                {
                    rx_buffer[rx_buffer_idx++] = uart_rx_byte; // 将字节存入缓冲区并移动索引
                }
                else
                {
                    // 缓冲区溢出处理：重置缓冲区，丢弃当前命令
                    rx_buffer_idx = 0;
                    // 可以在这里添加错误指示，例如点亮LED
                    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                }
            }
        }
    }
}

/**
  * @brief  解析UART接收到的命令字符串。
  *         期望的格式: "CAN_ID,CMD_TYPE,VALUE"
  * @param  cmd_str: 要解析的命令字符串。
  * @retval None
  */
void UartRxParser(const char *cmd_str)
{


    CanCmd_t can_cmd;
    uint32_t can_id_temp; // 临时变量，用于存储解析出的CAN ID
    char cmd_type_str[16]; // Max length for CMD_TYPE string
    int32_t value_temp; // 临时变量，用于存储解析出的CAN值

    // Parse the string using sscanf
    // 根据GCC的警告信息，uint32_t 对应 'long unsigned int'，应使用 %lu
    // int32_t 对应 'long int'，应使用 %ld
    if (sscanf(cmd_str, "%u,%15[^,],%d", &can_id_temp, cmd_type_str, &value_temp) == 3)
    {
        can_cmd.can_id = can_id_temp;
        can_cmd.cmd = GetCanCmdTypeFromString(cmd_type_str);
        can_cmd.value = value_temp;

        // 检查命令类型是否是已知的
        if (can_cmd.cmd != CMD_CAN_UNKNOWN)
        {
            // 将解析后的命令发送到CanCmdQueueHandle队列，等待CAN发送任务处理。
            // 0, 0 表示不等待，如果队列满则立即返回错误。
            if (osMessageQueuePut(CanCmdQueueHandle, &can_cmd, 0, 0) != osOK)
            {
                // 队列满错误处理，例如切换LED指示
            //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
        }
        else
        {
            // 未知命令类型错误处理
            // 
        }
    }
    else
    {
        // 字符串解析失败错误处理（例如格式不正确）
      //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/**
  * @brief  CAN发送任务。
  *         接收解析后的CAN命令，并通过CAN总线发送。
  * @param  argument: 未使用
  * @retval None
  */
void CanTxTask(void *argument)
{
    CanCmd_t can_cmd; // 用于存储从队列中获取的CAN命令
    CAN_TxHeaderTypeDef TxHeader; // CAN发送帧的头部结构体
    uint8_t TxData[8]; // CAN发送的数据缓冲区，CAN数据帧最多8个字节
    uint32_t TxMailbox; // 用于存储CAN发送邮箱的编号

    for (;;) // 任务的无限循环
    {
        // 尝试从CanCmdQueueHandle队列中获取一个CAN命令。
        // osWaitForever 表示如果队列为空，任务将一直等待直到有数据。
        if (osMessageQueueGet(CanCmdQueueHandle, &can_cmd, NULL, osWaitForever) == osOK)
        {
            // 配置CAN发送帧的头部
            TxHeader.StdId = can_cmd.can_id; // 设置标准CAN ID
            TxHeader.ExtId = 0x00; // 不使用扩展ID
            TxHeader.RTR = CAN_RTR_DATA; // 数据帧（不是远程请求帧）
            TxHeader.IDE = CAN_ID_STD; // 标准ID
            TxHeader.DLC = 8; // 数据长度为8个字节

            // 填充TxData数据缓冲区
            TxData[0] = (uint8_t)can_cmd.cmd; // 第一个字节存储命令类型
            // 使用 memcpy 将 int32_t 类型的值复制到 TxData 的第1个字节开始的4个字节中
            memcpy(&TxData[1], &can_cmd.value, sizeof(int32_t));

            // 如果数据长度是8，且值只占4个字节，则将剩余字节填充为0
            // 这里从 TxData[1 + sizeof(int32_t)] 开始填充，即从第5个字节开始
            for (int i = 1 + sizeof(int32_t); i < 8; i++) {
                TxData[i] = 0;
            }

            // 将CAN消息添加到CAN发送邮箱中
            // &hcan1 是CAN外设的句柄
            // &TxHeader 是CAN发送帧头部信息
            // TxData 是要发送的数据
            // &TxMailbox 是用于接收发送邮箱编号的变量
            if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
            {
                // 发送错误处理，调用错误处理函数
                Error_Handler();
            }
        }
    }
}

/**
  * @brief  将命令类型字符串转换为 CanCmdType_t 枚举值。
  * @param  cmd_type_str: 命令类型字符串 (例如, "SET_SPEED", "STOP")。
  * @retval CanCmdType_t 枚举值。
  */
static CanCmdType_t GetCanCmdTypeFromString(const char *cmd_type_str)
{
    // 使用 strcmp 比较字符串
    if (strcmp(cmd_type_str, "SET_SPEED") == 0)
    {
        return CMD_CAN_SET_SPEED;
    }
    else if (strcmp(cmd_type_str, "STOP") == 0)
    {
        return CMD_CAN_STOP;
    }
    // 如果没有匹配的命令类型，则返回未知命令类型
    return CMD_CAN_UNKNOWN;
}