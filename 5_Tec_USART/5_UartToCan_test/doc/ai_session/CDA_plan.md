# 5_UTC 网关 CAN 通信能力分析与修复计划

## 一、当前架构总览

```
上位机 (UART)
  │
  ├─ RX (中断) → RingBuffer[256] → ProtocolParser_Task (pri=Normal1)
  │                                    ↓
  │                               uartToCanQueue[16]
  │                                    ↓
  └─ TX (DMA) ←── uart1_send(mutex+sem) ←── UartToCan_Task (pri=Normal)
                                                   ↓
                                              HAL_CAN_AddTxMessage
                                                   ↓
                                              CAN TX Mailbox ×3
                                                   ↓
  MCLM1(M1,M2@50ms) ←─── CAN Bus @500kbps ───→ MCLM2(M1,M2@50ms)
                        4帧/50ms = 80帧/秒
                              ↓
                   CAN FIFO0 → ISR → canRxQueue[16]
                                         ↓
                              CanRxProcess_Task (pri=Normal)
                                         ↓
                                  uart1_send(DMA)
```

## 二、发现的问题

### 问题①：UART TX 互斥锁竞争（主因）

`UartToCan_Task_Run` 和 `CanRxProcess_Task_Run` **共用同一个** `uart1_send()`（DMA + mutex）。

```
CanRxProcess: [print状态帧 7ms]→[下一帧]→[print状态帧 7ms]→...
                  ↑持锁              锁竞争               ↑持锁
UartToCan:     [block on mutex...]   抢锁...   [终于拿到锁→print诊断→发CAN]
```

**后果**：上位机 UART 命令到达后，UartToCan 要等 CanRxProcess 完成当前状态帧的 UART 输出（~7ms），然后两者抢锁，可能再等几轮。**CAN 命令最终能发出，但延迟不确定**。

### 问题②：诊断打印在 CAN 发送之前

`app_task.c:UartToCan_Task_Run()` 当前顺序：

```c
// 第1步：诊断打印（持锁7ms）
uart1_send(dbg_buffer, offset);
// 第2步：发CAN
HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);
```

诊断打印在关键路径上，阻塞了 CAN 发送。

### 问题③：CAN RX 队列可能溢出

| 场景 | 帧率 | 处理能力 | 结论 |
|---|---|---|---|
| 2个MCLM（4帧/50ms） | 80帧/秒 | ~143帧/秒 | ✅ 安全 |
| 4个MCLM（8帧/50ms） | 160帧/秒 | ~143帧/秒 | ❌ 渐变满 |
| 突发短时峰值 | 更高 | ~143帧/秒 | ❌ 队列满 → 静默丢帧 |

**队列满时 ISR 中 `osMessageQueuePut` 返回失败但无人检查**，消息静默丢弃。

### 问题④：CAN TX 邮箱分析（非主因）

STM32F103 有 3 个 TX 邮箱，`AutoRetransmission = ENABLE`。只要总线上有其他节点（MCLM 发状态帧），所有 CAN 帧都会被 ACK，邮箱不会卡死。 **不是"无法下发"的主因。**

---

## 三、修复方案

### Fix 1：诊断打印移到 CAN 发送之后（高优先级）

**改动文件**：`App/app_task.c` — `UartToCan_Task_Run()`

```c
// 改成：先发CAN，再根据结果打印
HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);

// 诊断放在后面，不阻塞关键路径
if (tx_status != HAL_OK)
{
    int offset = sprintf(dbg_buffer, "UART->CAN | TX_FAIL | ID:0x%lX Status:%d\r\n",
                         uart_msg.id, tx_status);
    uart1_send(dbg_buffer, offset);
}
else
{
    // 可选：只在调试时打开成功打印
    int offset = sprintf(dbg_buffer, "UART->CAN | OK | ID:0x%lX\r\n", uart_msg.id);
    // uart1_send(dbg_buffer, offset);
}
```

**效果**：CAN 发送不再被 UART 诊断输出阻塞。

### Fix 2：CanRxProcess 输出精简（中优先级）

**改动文件**：`App/app_task.c` — `CanRxProcess_Task_Run()`

状态帧每 50ms 更新，全部打印到 UART 太占带宽。两种策略选一：

**策略A**：仅在有变化时打印（帧率不变但UART闲时静默）
**策略B**：降低打印频率（每5帧打一次，或每250ms打一次）

推荐策略A，用 flags 和 speed 是否变化来判断：

```c
static int16_t last_speed[4] = {0};  // 按CAN ID索引
static uint8_t last_flags[4] = {0};

// 仅在速度或flags变化时打印
bool changed = (current_speed != last_speed[idx] || flags != last_flags[idx]);
last_speed[idx] = current_speed;
last_flags[idx] = flags;

if (changed) {
    // 打印详细状态
} else {
    // 跳过或只打一行简略心跳
}
```

**效果**：UART TX 负载从 7ms/帧 降至接近 0（稳态时）。

### Fix 3：CAN RX 队列溢出检测（低优先级）

在 `stm32f1xx_it.c:HAL_CAN_RxFifo0MsgPendingCallback()` 中检查 `osMessageQueuePut` 返回值：

```c
if (osMessageQueuePut(canRxQueueHandle, &can_msg, 0, 0) != osOK)
{
    // 队列满，递增丢帧计数器（原子操作）
    __atomic_fetch_add(&g_can_rx_drop_count, 1, __ATOMIC_RELAXED);
}
```

并在 `CanRxProcess_Task_Run` 中定期报告：

```c
if (g_can_rx_drop_count > 0) {
    int n = g_can_rx_drop_count;
    g_can_rx_drop_count = 0;
    sprintf(dbg, "WARN: %d CAN frames dropped (queue full)\r\n", n);
    uart1_send(dbg, ...);
}
```

---

## 四、优先级排序

| 顺序 | 修复项 | 工作量 | 影响 |
|---|---|---|---|
| P0 | Fix 1：诊断打印移到CAN发送之后 | 改5行 | 消除"无法下发"延迟 |
| P1 | Fix 2：CanRxProcess 输出精简 | 改15行 | 释放UART带宽，消除队列溢出根源 |
| P2 | Fix 3：增加丢帧检测 | 改10行 | 可观测性，便于调试 |

---

## 五、Fix 2 详细设计

### 按ID跟踪上次状态

```c
// CanRxProcess_Task_Run 内，静态变量
#define STATUS_ID_COUNT 4
static int16_t  prev_speed[STATUS_ID_COUNT];
static uint8_t  prev_flags[STATUS_ID_COUNT];
static uint32_t last_print_tick[STATUS_ID_COUNT];
static bool     first_frame[STATUS_ID_COUNT] = {true, true, true, true};

// ID→索引映射
int map_id_to_index(uint32_t id) {
    switch (id) {
        case 0x323: return 0;
        case 0x324: return 1;
        case 0x325: return 2;
        case 0x326: return 3;
        default:    return -1;
    }
}
```

### 打印策略

- **首次收到**某 ID 的帧：全量打印
- **速度或 flags 变化**：全量打印
- **连续不变超过 500ms**：打印一次心跳简略行
- **其余**：跳过打印
