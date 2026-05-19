# 5_UTC 数据流文档

> 基于源码梳理 `5_UartToCan_test` 桥接器的完整数据流。

---

## 系统概述

`5_UartToCan_test` 是一个 **UART ↔ CAN 双向桥接器**，运行于 STM32F103C8T6。

```
PC (串口终端)
    ↕ UART1 (PA9 TX, PA10 RX, 115200 8N1)
5_UartToCan_test (桥接器)
    ↕ CAN1 (PA11 RX, PA12 TX, 500kbps)
3_MCLM_t2 (电机控制器 Group1/Group2)
```

每组电机控制器控制两台电机：转向(TURN) + 动力(POWER)。

---

## 硬件资源

| 外设 | 引脚 | 速率 | 用途 |
|------|------|------|------|
| USART1 | PA9(TX), PA10(RX) | 115200 8N1 | 与 PC 通信 |
| USART2 | PA2(TX), PA3(RX) | 115200 8N1 | 已初始化，应用层未使用 |
| CAN1 | PA11(RX), PA12(TX) | 500kbps | 与电机控制器通信 |
| PC13 | 板载 LED | - | 心跳指示 (300ms 翻转) |

CAN 波特率计算: APB1=36MHz, Prescaler=4, BS1=13, BS2=4 → 36M/4/(1+13+4)=500kbps

---

## 上行数据流：PC → UART → CAN → 目标机

PC 通过串口发送自定义协议帧，桥接器解析后转发为 CAN 帧到电机控制器。

### 第 1 段：UART 接收 (ISR)

```
USART1 RX 中断
  → HAL_UART_RxCpltCallback()                    [stm32f1xx_it.c:87]
    → ring_buffer_put(&uart1_rx_buffer, byte)     [usart.c:293] 单字节存入环形缓冲区
    → osEventFlagsSet(uart1_rx_eventHandle)       通知 ProtocolParser 任务
    → HAL_UART_Receive_IT(···, 1)                  重新启动单字节中断接收
```

- 每次中断只接收 1 字节，存入 256 字节环形缓冲区 `uart1_rx_buffer`
- 缓冲区使用 `uint8_t head/tail` 天然溢出回绕，**无需临界区保护** (ARM Cortex-M3 单字节读写原子)

### 第 2 段：协议解析 (任务)

```
ProtocolParser_Task_Run()                          [app_task.c:194]
  优先级: osPriorityNormal1 (= 53)
  栈: 1024 字节 (256 * uint32_t)
```

有限状态机解析 UART 协议帧：

```
状态机: WAIT_SOF → WAIT_CMD → WAIT_ID → WAIT_LEN → WAIT_DATA → (完成)
```

**UART 协议帧格式：**

| 字段 | 大小 | 说明 |
|------|------|------|
| SOF | 1B | 固定 `0xAA` (Start of Frame) |
| CMD | 1B | 指令 (Command_ID_t: 0x01=调速, 0x02=查状态, 0x03=设模式, 0x04=急停) |
| ID | 4B LE | CAN ID (小端，支持 29 位扩展帧) |
| LEN | 1B | DATA 长度 (0~8) |
| DATA | LEN B | CAN 数据负载 |

**容错处理：** 任何状态下收到 `0xAA` (SOF) 即重置状态机重新开始，防止误同步。

解析完成后，结果通过 `osMessageQueuePut(uartToCanQueueHandle)` 送入 `uartToCanQueue` (16 槽，元素类型 `App_UART_Message_t`，sizeof=16)。

### 第 3 段：UART → CAN 转发 (任务)

```
UartToCan_Task_Run()                               [app_task.c:52]
  优先级: osPriorityNormal (= 52)
  栈: 2048 字节 (512 * uint32_t)
```

1. 阻塞等待 `uartToCanQueue` 消息 (`osMessageQueueGet`, 永久等待)
2. 判断 ID 是否 > 0x7FF → 选标准帧或扩展帧
3. 设置 CAN Tx Header: DLC=len, IDE,RTR,ID
4. `HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox)` 发送
5. 发送失败时打印诊断信息（通过 UART1 回显）

---

## 下行数据流：目标机 → CAN → UART → PC

电机控制器通过 CAN 发送状态帧，桥接器接收并格式化为字符串转发到 PC 串口终端。

### 第 1 段：CAN 接收 (ISR)

```
CAN1 RX FIFO0 中断
  → HAL_CAN_RxFifo0MsgPendingCallback()            [stm32f1xx_it.c:61]
    → HAL_CAN_GetRxMessage()                       从硬件 FIFO0 读取
    → 打包为 App_CAN_Message_t { id, len, data[8] }
    → osMessageQueuePut(canRxQueueHandle)           送入 canRxQueue
```

- CAN 过滤器：全通 (ID=0x0000, Mask=0x0000)，接收所有报文
- 从 ISR 调用 `osMessageQueuePut` 必须传 timeout=0

### 第 2 段：CAN 接收处理 (任务)

```
CanRxProcess_Task_Run()                            [app_task.c:107]
  优先级: osPriorityNormal (= 52)
  栈: 2048 字节 (512 * uint32_t)
```

1. 阻塞等待 `canRxQueue` 消息
2. **判断是否为电机状态帧** (ID 匹配 0x323/0x324/0x325/0x326)：

   **帧格式解码：**
   | 偏移 | 类型 | 字段 | 参考结构体 |
   |------|------|------|-----------|
   | [0-1] | int16 LE | `current_logic_speed` | Motor_t.current_logic_speed |
   | [2-3] | uint16 LE | `accumulated_ticks` | Motor_t.accumulated_ticks |
   | [4-5] | int16 LE | `pwm_output` | Motor_t.pwm_output |
   | [6] | int8 | `target_logic_speed` | Motor_t.target_logic_speed |
   | [7] | uint8 | `flags` | Motor_t.flags |

   **flags 解码：** `0x01`=堵转(STALL), `0x02`=PID饱和(SAT)

   输出示例：
   ```
   CAN RX | TURN(G2) STATUS | curr=123 accum=456 pwm=789 target=50 flags=0x00
   CAN RX | POWER(G1) STATUS | curr=-50 accum=1234 pwm=-200 target=-30 flags=0x01 STALL
   ```

3. **非状态帧**：按 hex dump 格式输出:
   ```
   CAN RX | ID: 0x123 | DLC: 8 | Data: AA 01 00 00 00 08 64 00
   ```

### 第 3 段：UART TX 发送

```
uart1_send(buf, len)                                [app_task.c:32]
```

- 互斥锁 `uart1_tx_mutexHandle` 防止多任务并发发送
- 数据拷贝到静态 DMA 缓冲区 `uart1_tx_dma_buf[128]`
- `HAL_UART_Transmit_DMA()` (DMA1 Channel4, Normal 模式)
- 信号量 `uart1_tx_semHandle` 等待 `HAL_UART_TxCpltCallback` 释放
- 释放互斥锁

---

## FreeRTOS 任务一览

| 任务名 | 函数 | 优先级 | 栈 (uint32_t) | 角色 |
|--------|------|--------|---------------|------|
| `ProtocolParser_` | `ProtocolParser_Task_Run` | Normal1 (53) | 256 (1KB) | UART 协议解析状态机 |
| `UartToCan_Ta` | `UartToCan_Task_Run` | Normal (52) | 512 (2KB) | CAN 发送 |
| `CanRxProcess_Ta` | `CanRxProcess_Task_Run` | Normal (52) | 512 (2KB) | CAN 接收处理 + UART TX |
| `Heartbeat_Ta` | `Heartbeat_Task_Run` | Low (8) | 64 (256B) | LED 心跳 |

## IPC 与同步对象

| 对象 | 类型 | 容量 | 元素大小 | 用途 |
|------|------|------|---------|------|
| `uartToCanQueue` | MessageQueue | 16 | `App_UART_Message_t` (16B) | Parser → CAN TX |
| `canRxQueue` | MessageQueue | 16 | `App_CAN_Message_t` (16B) | CAN ISR → Rx Process |
| `uart1_tx_mutex` | Mutex | - | - | 保护 UART1 TX |
| `uart1_tx_sem` | Semaphore | 1 | - | DMA TX 完成同步 |
| `uart1_rx_event` | EventFlags | 32-bit | - | UART RX ISR → Parser 通知 |

## 数据流总图

```
┌──────────────┐
│   PC 串口    │
│  (串口终端)   │
└──┬───────┬───┘
   │ UART  │ UART
   │  RX   │  TX
   ▼       ▲
┌──────────────┐
│ ring_buffer  │ uart1_send()
│ uart1_rx_buf │ (DMA TX)
└──────┬───────┘
       │ osEventFlags
       ▼
┌──────────────────┐
│ ProtocolParser_  │  UART 协议状态机 (WAIT_SOF→CMD→ID→LEN→DATA)
│ (Task, pri=53)   │
└──────┬───────────┘
       │ osMessageQueuePut
       ▼
┌──────────────────┐
│ uartToCanQueue   │  (16 槽)
└──────┬───────────┘
       │ osMessageQueueGet
       ▼
┌──────────────────┐
│ UartToCan_Task   │  CAN TX
│ (Task, pri=52)   │
└──────┬───────────┘
       │ HAL_CAN_AddTxMessage
       ▼
   ┌────────┐
   │ CAN 总线│  (500kbps)
   └──┬─────┘
      │ CAN RX ISR
      ▼
┌──────────────────┐
│ canRxQueue       │  (16 槽)
└──────┬───────────┘
       │ osMessageQueueGet
       ▼
┌──────────────────┐
│ CanRxProcess_Ta  │  CAN→UART 解码转发
│ (Task, pri=52)   │
└──────┬───────────┘
       │ uart1_send() (DMA)
       ▼
┌──────────────┐
│   PC 串口    │
└──────────────┘
```

## CAN ID 分配

| 宏 | ID | 方向 | 组 | 用途 |
|---|----|------|-----|------|
| `CAN_MOTOR_TURN_CMD_STDID_G2` | **0x123** | RX | G2 | 转向电机控制指令 |
| `CAN_MOTOR_POWER_CMD_STDID_G2` | **0x124** | RX | G2 | 动力电机控制指令 |
| `CAN_MOTOR_TURN_CMD_STATUS_STDID_G2` | **0x223** | RX | G2 | 转向电机状态查询 |
| `CAN_MOTOR_POWER_CMD_STATUS_STDID_G2` | **0x224** | RX | G2 | 动力电机状态查询 |
| `CAN_MOTOR_TURN_STATUS_STDID_G2` | **0x323** | TX | G2 | 转向电机状态反馈 |
| `CAN_MOTOR_POWER_STATUS_STDID_G2` | **0x324** | TX | G2 | 动力电机状态反馈 |
| `CAN_MOTOR_TURN_CMD_STDID_G1` | **0x125** | RX | G1 | 转向电机控制指令 |
| `CAN_MOTOR_POWER_CMD_STDID_G1` | **0x126** | RX | G1 | 动力电机控制指令 |
| `CAN_MOTOR_TURN_CMD_STATUS_STDID_G1` | **0x225** | RX | G1 | 转向电机状态查询 |
| `CAN_MOTOR_POWER_CMD_STATUS_STDID_G1` | **0x226** | RX | G1 | 动力电机状态查询 |
| `CAN_MOTOR_TURN_STATUS_STDID_G1` | **0x325** | TX | G1 | 转向电机状态反馈 |
| `CAN_MOTOR_POWER_STATUS_STDID_G1` | **0x326** | TX | G1 | 动力电机状态反馈 |

**ID 编码规律：**
- `0x1xx` = 控制指令 (RX for 桥接器, TX for PC)
- `0x2xx` = 状态查询 (RX for 桥接器)
- `0x3xx` = 状态反馈 (TX for 桥接器, RX for PC)
- `x23` = Group 2 (TURN=3, POWER=4)
- `x25` = Group 1 (TURN=5, POWER=6)

## 初始化顺序

```
main():
  1. HAL_Init()
  2. SystemClock_Config()       → 72MHz HSE+PLL
  3. MX_GPIO_Init()             → GPIO
  4. MX_DMA_Init()              → DMA1 Ch4(TX)/Ch5(RX)/Ch6(UR2 RX)/Ch7(UR2 TX)
  5. MX_CAN_Init()              → CAN1 500kbps Normal
  6. MX_USART1_UART_Init()      → USART1 115200 DMA+IT
  7. MX_USART2_UART_Init()      → USART2 115200 (保留)
  8. CAN_Filter_Config()        → 全通过滤器
  9. HAL_CAN_Start()            → 启动 CAN
  10. HAL_CAN_ActivateNotification() → 使能 RX FIFO0 中断
  11. UART_Receive_Start()      → 初始化环形缓冲区 + 启动 UART IT RX
  12. osKernelInitialize()
  13. MX_FREERTOS_Init()        → 创建队列/互斥锁/信号量/事件/任务
  14. osKernelStart()           → 启动调度器
```
