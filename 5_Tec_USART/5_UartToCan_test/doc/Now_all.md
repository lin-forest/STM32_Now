# UartToCan_test 工程架构与数据流分析

---

## 一、工程文件职能总览

```
App/
├── app_config.h      ── 协议常量、数据结构类型定义（全局类型中枢）
├── app_globals.h     ── 跨模块共享句柄的 extern 声明
├── app_includes.h    ── 统一总头文件（单点包含入口）
├── ring_buffer.h/c   ── 无锁环形缓冲区（ISR↔Task 数据桥）
└── app_task.h/c      ── 四个 FreeRTOS 任务的全部实现

doc/
├── trae.md           ── 架构与数据流文档（主设计文档）
├── claude.md         ── 代码审查优化归档（P0/P1/P2 问题记录）
├── fix1_UartToDma.md ── UART TX 阻塞→DMA 改造专题记录
├── result.md         ── 项目目标完成状态
├── goal.md           ── 演进规划与验证要点
└── Now_all.md        ── 当前状态汇总（本文件）
```

---

## 二、分层架构

```
┌──────────────────────────────────────────────────────┐
│                  应用逻辑层 (App/)                     │
│   ProtocolParser  UartToCan  CanRxProcess  Heartbeat  │
├──────────────────────────────────────────────────────┤
│               操作系统层 (FreeRTOS)                    │
│   Queue  Mutex  Semaphore  EventFlags  osDelay        │
├──────────────────────────────────────────────────────┤
│              硬件抽象层 (STM32 HAL)                    │
│   USART1(DMA)  CAN  GPIO(PC13)  DMA                  │
└──────────────────────────────────────────────────────┘
```

---

## 三、各文件职能详解

### 3.1 `app_config.h` — 配置与类型定义

| 元素 | 类型 | 作用 |
|---|---|---|
| `FRAME_SOF 0xAA` | 宏 | 自定义协议帧头标志字节 |
| `CAN_RX_QUEUE_SIZE 16` | 宏 | CAN 接收队列容量 |
| `UART_TO_CAN_QUEUE_SIZE 16` | 宏 | UART→CAN 转发队列容量 |
| `Command_ID_t` | enum | 指令集：CMD_SET_SPEED / CMD_GET_STATE / CMD_SET_MODE / CMD_ESTOP |
| `App_UART_Message_t` | struct | UART 解析结果（id + cmd + len + data[8]），入 `uartToCanQueue` |
| `App_CAN_Message_t` | struct | CAN 接收报文（id + len + data[8]），入 `canRxQueue` |
| `ParserState_t` | enum | 状态机五个状态：STATE_WAIT_SOF / STATE_WAIT_CMD / STATE_WAIT_ID / STATE_WAIT_LEN / STATE_WAIT_DATA |

> `App_UART_Message_t` 中 `uint32_t id` 优先排列，结构体大小 16 字节，无填充浪费。

---

### 3.2 `app_globals.h` — 全局句柄声明

| 变量 | 类型 | 作用 |
|---|---|---|
| `hcan` | `CAN_HandleTypeDef` | HAL CAN 外设句柄 |
| `huart1` | `UART_HandleTypeDef` | USART1（调试/协议口）句柄 |
| `huart2` | `UART_HandleTypeDef` | USART2（备用）句柄 |
| `canRxQueueHandle` | `osMessageQueueId_t` | CAN ISR → CanRxProcess 任务的消息队列 |
| `uartToCanQueueHandle` | `osMessageQueueId_t` | ProtocolParser → UartToCan 任务的消息队列 |
| `uart1_tx_mutexHandle` | `osMutexId_t` | 保护 UART1 TX 通道，防多任务并发乱码 |
| `uart1_tx_semHandle` | `osSemaphoreId_t` | DMA 传输完成信号量（ISR 释放，任务等待） |
| `uart1_rx_eventHandle` | `osEventFlagsId_t` | ISR 通知 ProtocolParser 有新数据到来 |
| `UART1_RX_FLAG 0x01U` | 宏 | 事件标志位掩码 |

---

### 3.3 `ring_buffer.h / ring_buffer.c` — 无锁环形缓冲区

> 设计精髓：利用 `uint8_t` 自然溢出实现无锁 SPSC（单写单读）缓冲。

| 元素 | 说明 |
|---|---|
| `UART_RX_BUFFER_SIZE 256` | 固定 256，`uint8_t` 溢出回绕 ≡ `% 256`，无需取模运算 |
| `volatile uint8_t head` | ISR 写，`volatile` 防止编译器寄存器缓存 |
| `volatile uint8_t tail` | 任务读，单字节操作在 Cortex-M3 天然原子 |
| `ring_buffer_init()` | `head = tail = 0`，清空缓冲区 |
| `ring_buffer_put()` | ISR 调用：`next_head == tail` 则满，返回 `false` 丢弃 |
| `ring_buffer_get()` | 任务调用：`head == tail` 则空，返回 `false` |

```
buffer[0..255]:  [ ... data bytes ... ]
                    ↑tail(读)    ↑head(写)
  ISR  向 head 写入，head++（uint8_t 自动回绕）
  Task 从 tail 读取，tail++（uint8_t 自动回绕）
  满判断：(head + 1) == tail
  空判断：head == tail
```

---

### 3.4 `app_task.h` — 任务接口声明

纯声明四个任务函数，供 `freertos.c`（CubeMX 生成）在创建任务时注册：

| 函数 | 绑定任务 |
|---|---|
| `ProtocolParser_Task_Run()` | `ProtocolParser_Ta`（High 优先级） |
| `UartToCan_Task_Run()` | `UartToCan_Ta`（Normal 优先级） |
| `CanRxProcess_Task_Run()` | `CanRxProcess_Ta`（Normal 优先级） |
| `Heartbeat_Task_Run()` | `Heartbeat_Ta`（Low 优先级） |

---

### 3.5 `app_task.c` — 核心任务实现

#### 静态辅助函数：`uart1_send(buf, len)` — DMA 发送封装

```
调用方
  │  osMutexAcquire(uart1_tx_mutexHandle)    ← 独占 TX 通道
  │  memcpy → uart1_tx_dma_buf[128]           ← 拷贝到静态缓冲（DMA 需持续有效）
  │  HAL_UART_Transmit_DMA()                  ← 启动 DMA，立即返回
  │  osSemaphoreAcquire(uart1_tx_semHandle)   ← 挂起，等待 TxCplt 回调
  ↓                                              ↑ ISR 中 osSemaphoreRelease()
  osMutexRelease(uart1_tx_mutexHandle)        ← 释放通道
```

关键变量：

| 变量 | 说明 |
|---|---|
| `uart1_tx_dma_buf[128]` | `static` 静态数组，生命周期贯穿整个 DMA 传输 |
| `UART1_TX_DMA_BUF_SIZE 128` | 最大单次发送长度限制 |

---

#### 任务①：`ProtocolParser_Task_Run()` — 协议解析（最高优先级）

状态机驱动，局部关键变量：

| 变量 | 类型 | 作用 |
|---|---|---|
| `state` | `ParserState_t` | 当前状态机状态 |
| `current_msg` | `App_UART_Message_t` | 正在组装中的帧 |
| `byte_received` | `uint8_t` | 从环形缓冲区取出的单字节 |
| `id_byte_count` | `uint8_t` | 已接收的 CAN_ID 字节数（累计到 4） |
| `data_idx` | `uint8_t` | 已接收的数据负载字节数 |

状态机逻辑（每次取出 1 字节驱动）：

```
STATE_WAIT_SOF  → 检测 0xAA          → STATE_WAIT_CMD
STATE_WAIT_CMD  → 存 cmd             → STATE_WAIT_ID
STATE_WAIT_ID   → 小端拼接 4 字节 id  → STATE_WAIT_LEN
STATE_WAIT_LEN  → 检查 len ≤ 8       → STATE_WAIT_DATA（len==0 直接入队）
STATE_WAIT_DATA → 填 data[]          → 满足 len 后 osMessageQueuePut → STATE_WAIT_SOF

任意状态收到 0xAA → 视为新帧开始，重置并跳回 STATE_WAIT_CMD（容错）
```

> **阻塞机制**：缓冲区为空时调用 `osEventFlagsWait`（10 ms 超时），ISR 写入后立即 `osEventFlagsSet` 唤醒，近零延迟。

---

#### 任务②：`UartToCan_Task_Run()` — UART→CAN 转换

| 局部变量 | 作用 |
|---|---|
| `uart_msg`（`App_UART_Message_t`） | 从 `uartToCanQueueHandle` 取出的消息 |
| `tx_header`（`CAN_TxHeaderTypeDef`） | HAL CAN 发送帧头（IDE / RTR / DLC / ID） |
| `tx_mailbox`（`uint32_t`） | HAL 分配的 CAN 发送邮箱编号 |
| `dbg_buffer[128]` | `sprintf` 诊断字符串暂存 |

帧类型自动判断：

```c
if (uart_msg.id > 0x7FF)   // 扩展帧 29-bit
    tx_header.IDE = CAN_ID_EXT;  tx_header.ExtId = uart_msg.id;
else                        // 标准帧 11-bit
    tx_header.IDE = CAN_ID_STD;  tx_header.StdId = uart_msg.id;
```

---

#### 任务③：`CanRxProcess_Task_Run()` — CAN→UART 透明输出

| 局部变量 | 作用 |
|---|---|
| `rx_can_msg`（`App_CAN_Message_t`） | 从 `canRxQueueHandle` 取出的 CAN 报文 |
| `tx_buffer[128]` | 格式化输出字符串缓冲区 |

格式化输出：`"CAN RX | ID: 0x%03lX | DLC: %d | Data: XX XX ...\r\n"`，通过 `uart1_send()` 加锁发送。

---

#### 任务④：`Heartbeat_Task_Run()` — 心跳

```c
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // 翻转板载 LED
osDelay(300);                             // 300 ms 周期
```

> 系统存活指示：LED 停止闪烁 = 任务死锁或崩溃。

---

## 四、完整数据流图

### 4.1 上行：UART → CAN

```
[PC 串口工具]
    │  自定义帧: AA | CMD | ID(4B, LE) | DLC | Data
    ▼
USART1 外设（中断接收）
    │
    ▼  HAL_UART_RxCpltCallback（usart.c / ISR 上下文）
    │  ├─ ring_buffer_put(&uart1_rx_buffer, byte)        写 head
    │  └─ osEventFlagsSet(uart1_rx_eventHandle, 0x01)    唤醒解析任务
    ▼
uart1_rx_buffer [RingBuffer_t, 256B]  ← ISR 写 head / 任务读 tail，无锁
    │
    ▼  ProtocolParser_Task_Run（High 优先级）
    │  ring_buffer_get() 逐字节取出
    │  状态机解析 → 组装 App_UART_Message_t
    │  osMessageQueuePut(uartToCanQueueHandle, &current_msg)
    ▼
uartToCanQueue [容量 16 × App_UART_Message_t]
    │
    ▼  UartToCan_Task_Run（Normal 优先级）
    │  osMessageQueueGet() 阻塞等待
    │  判断帧类型（> 0x7FF → 扩展帧），填充 tx_header
    │  HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox)
    ▼
[CAN 总线]
```

### 4.2 下行：CAN → UART

```
[CAN 总线]
    │
    ▼  USB_LP_CAN1_RX0_IRQHandler（ISR 上下文）
    │  封装 App_CAN_Message_t（id / len / data）
    │  osMessageQueuePut(canRxQueueHandle, &msg)
    ▼
canRxQueue [容量 16 × App_CAN_Message_t]
    │
    ▼  CanRxProcess_Task_Run（Normal 优先级）
    │  osMessageQueueGet() 阻塞等待
    │  sprintf 格式化字符串到 tx_buffer[128]
    │  uart1_send(tx_buffer, len)
    │      ├─ osMutexAcquire(uart1_tx_mutexHandle)   独占 TX 通道
    │      ├─ memcpy → uart1_tx_dma_buf[128]          静态缓冲
    │      ├─ HAL_UART_Transmit_DMA()                 启动 DMA
    │      ├─ osSemaphoreAcquire(uart1_tx_semHandle)  挂起等完成
    │      │       ↑ HAL_UART_TxCpltCallback → osSemaphoreRelease
    │      └─ osMutexRelease(uart1_tx_mutexHandle)
    ▼
USART1 TX → [PC 串口工具]
```

---

## 五、关键同步机制汇总

| 同步对象 | 方向 | 生产者 | 消费者 | 解决问题 |
|---|---|---|---|---|
| `uart1_rx_buffer`（RingBuffer） | ISR→Task | UART RxISR | ProtocolParser | 字节接收缓冲，无锁 SPSC |
| `uart1_rx_eventHandle`（EventFlag） | ISR→Task | UART RxISR | ProtocolParser | 事件驱动唤醒，零延迟 |
| `uartToCanQueue`（Queue×16） | Task→Task | ProtocolParser | UartToCan | 帧级解耦，背压保护 |
| `canRxQueue`（Queue×16） | ISR→Task | CAN RxISR | CanRxProcess | CAN 报文缓冲，ISR 快速返回 |
| `uart1_tx_mutexHandle`（Mutex） | 多→单 | — | `uart1_send()` | 防两个任务同时操作 UART1 TX |
| `uart1_tx_semHandle`（Semaphore） | ISR→Task | TxCplt 回调 | `uart1_send()` | DMA 完成通知，任务让出 CPU |
