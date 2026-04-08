# 项目架构与数据流分析

本文档详细分析了 `UartToCan_test` 项目的软件架构、核心任务以及关键数据流。

## 1. 整体架构

项目基于 **STM32F103C8T6** 微控制器和 **FreeRTOS** 实时操作系统，采用模块化的多任务设计。核心功能是实现 UART (串口) 和 CAN (控制器局域网) 总线之间的双向数据转换。

其架构主要分为以下几个层次：

- **硬件抽象层 (HAL)**: 由 STMicroelectronics 提供的 `STM32F1xx_HAL_Driver`，负责底层硬件（如 UART, CAN, GPIO）的驱动和操作。
- **操作系统层 (OS)**: 使用 `FreeRTOS` 进行任务调度、同步和通信，保证了系统的实时性和稳定性。
- **应用逻辑层 (App)**: 包含了项目的核心业务逻辑，分为多个独立任务，通过消息队列进行解耦和通信。

## 2. 核心任务 (FreeRTOS Tasks)

系统共创建了四个核心任务，各司其职：

| 任务名称              | 优先级 | 核心功能                                                                                             |
| --------------------- | -------- | ---------------------------------------------------------------------------------------------------- |
| `ProtocolParser_`     | High     | **(关键)** 负责从UART接收原始字节流，通过状态机解析自定义协议，并将解析后的结构化消息发送到队列。 |
| `UartToCan_Ta`        | Normal   | 从队列中获取解析后的UART消息，将其打包成CAN报文，并通过CAN总线发送出去。                               |
| `CanRxProcess_Ta`     | Normal   | 从队列中获取CAN总线接收到的报文，将其格式化为人类可读的字符串，并通过UART发送出去。                  |
| `Heartbeat_Ta`        | Low      | 周期性地翻转板载LED（PC13），用于直观地指示系统是否正常运行。                                        |

## 3. 关键数据结构与组件

- **`ring_buffer` (环形缓冲区)**:
  - **文件**: `App/ring_buffer.c`
  - **用途**: 作为 UART 接收中断和 `ProtocolParser_` 任务之间的缓冲。中断服务程序（ISR）将接收到的字节快速存入缓冲区，而协议解析任务则从缓冲区安全地读取数据，实现了“中断-任务”模型，避免了在中断中执行耗时操作。

- **`canRxQueue` (消息队列)**:
  - **元素类型**: `App_CAN_Message_t`
  - **用途**: 作为 CAN 接收中断和 `CanRxProcess_Ta` 任务之间的桥梁。CAN 中断在收到报文后，将其打包成消息放入此队列，由处理任务取出并处理。

- **`uartToCanQueue` (消息队列)**:
  - **元素类型**: `App_UART_Message_t`
  - **用途**: `ProtocolParser_` 任务和 `UartToCan_Ta` 任务之间的通信管道。协议解析任务将解析好的有效数据帧放入此队列，等待被转换成 CAN 报文。

## 4. 数据流分析

### 4.1 上行数据流 (UART -> CAN)

此流程负责将通过串口发送的自定义协议数据转换为标准的 CAN 报文。

1.  **硬件接收**: `USART1` 外设接收到原始字节数据。
2.  **中断服务**: `USART1_IRQHandler` (在 `stm32f1xx_it.c`) 被触发，将接收到的字节存入 `uart1_rx_buffer` 环形缓冲区。
3.  **协议解析**: `ProtocolParser_Task_Run` 任务不断从 `uart1_rx_buffer` 读取字节，并通过一个状态机来解析自定义协议。
    - **自定义协议格式**: `SOF (1B)` + `CMD (1B)` + `CAN ID (4B)` + `DLC (1B)` + `Data (0-8B)`
4.  **入队**: 当一个完整的数据帧被成功解析后，它被封装成 `App_UART_Message_t` 结构体，并通过 `osMessageQueuePut` 放入 `uartToCanQueueHandle` 队列。
5.  **CAN 报文生成**: `UartToCan_Task_Run` 任务从 `uartToCanQueueHandle` 队列中取出消息。
6.  **智能帧类型判断**: 该任务会自动检查 CAN ID 的值。如果 ID 大于 `0x7FF`，则将其识别为 **扩展帧 (29-bit ID)**；否则，识别为 **标准帧 (11-bit ID)**。
7.  **硬件发送**: 任务调用 `HAL_CAN_AddTxMessage` 函数，将最终的 CAN 报文通过 CAN 控制器发送到总线上。

### 4.2 下行数据流 (CAN -> UART)

此流程负责监听 CAN 总线，并将接收到的报文信息通过串口打印出来。

1.  **硬件接收**: CAN 控制器接收到符合其过滤器设置的 CAN 报文。
2.  **中断服务**: `USB_LP_CAN1_RX0_IRQHandler` (在 `stm32f1xx_it.c`) 被触发。中断服务程序从 CAN 硬件读取报文，并将其封装成 `App_CAN_Message_t` 结构体。
3.  **入队**: 中断服务程序通过 `osMessageQueuePut` 将消息快速放入 `canRxQueueHandle` 队列。
4.  **出队与处理**: `CanRxProcess_Task_Run` 任务从 `canRxQueueHandle` 队列中取出 CAN 消息。
5.  **格式化输出**: 任务将 CAN 报文的 ID, DLC 和数据部分格式化成一个人类可读的字符串。
6.  **硬件发送**: 最终，该字符串通过 `HAL_UART_Transmit` 函数由 `USART1` 发送出去。

## 5. 诊断与调试

项目内置了有效的诊断机制：

- **心跳LED**: `Heartbeat_Ta` 任务提供了直观的系统运行状态指示。
- **串口日志**: `UartToCan_Ta` 和 `CanRxProcess_Ta` 任务都通过 `USART1` 打印关键的运行时信息，例如：
  - `UART->CAN` 转换时收到的指令内容。
  - CAN 报文发送失败时的错误码。
  - `CAN->UART` 转换时收到的完整报文内容。

这些设计极大地简化了调试过程。