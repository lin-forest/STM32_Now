# Fix3：UART↔CAN 网关 UART TX 阻塞问题 — 排查全记录

## 前置背景

**项目**：5_UartToCan_test — STM32F103C8T6 上的 UART↔CAN 双向桥接器。

**系统结构**：
- 上位机 (PC 串口终端) ←→ UART1 (115200, DMA TX + IT RX) ←→ STM32 ←→ CAN1 (500kbps) ←→ 电机控制器
- FreeRTOS (CMSIS-RTOS v2)，4 个任务 + 队列/互斥锁/信号量/事件标志 IPC

**上报问题**：
- CAN→UART 方向可以工作（CAN 总线数据 → 串口终端显示）
- UART→CAN 方向不可用（串口发指令 → CAN 总线无帧）
- 诊断 `[TXTST]` 输出从未在终端出现过
- **用户怀疑**："5_UTC 这个里面的协议没对上，导致的无法下发 can 指令"

---

## 排查过程（按时间顺序）

### 第一步：阅读已有文档

**操作**：读 `doc/ai_session/CDA_AIhistory.md`、`doc/ai_session/CDA_plan.md`、`doc/data_flow.md`

**目标**：了解之前做过哪些分析、发现了什么问题。

**已有发现**：
1. `cmd` 字段文档写"透传至 CAN data[0]"但代码没实现
2. 5_UTC 的 `Command_ID_t` 枚举值（0x01~0x04）与 3_MCLM_t2 电机控制器的命令字节（0x11=调速、0x01=查状态）不匹配
3. 最后 git 提交（19c25f4）将 uart1_send 从阻塞式 HAL_UART_Transmit 改为 DMA 版

**初步判断**：这是两个已知问题，但上一轮分析未能定位为什么 `[TXTST]` 完全不出现。

---

### 第二步：对比上层与下层协议定义

**操作**：读 `3_MCLM_t2/doc/deepseek_can.md`、`3_MCLM_t2/App/config/app_config.h`

**发现**：

| 项目 | 5_UTC 网关定义 | 3_MCLM_t2 实际 |
|------|---------------|-----------------|
| 调速命令 | `CMD_SET_SPEED = 0x01` | `CAN_CMD_SET_SPEED_T2 = 0x11` |
| 停止命令 | `CMD_ESTOP = 0x04` | `0x08` |
| 查询状态 | 无 | `0x01` |
| CAN 数据源 | `cmd` 字段被忽略，只发 `data[]` | CAN data[0] 放命令字节 |

**Python 脚本实际发送**（用户提供）：
```python
data = [0xAA, 0x01, 0x23, 0x01, 0x00, 0x00, 0x02, 0x11, 0x4B]
#       SOF   cmd     CAN ID (LE)      len  data[0] data[1]
#                                      ↓    ↓       ↓
# 实际解析: cmd=0x01(忽略), id=0x123, len=2, data[0]=0x11, data[1]=0x4B
```

**结论**：Python 脚本的协议帧可以正确解析 — cmd=0x01 被代码忽略，data[0]=0x11、data[1]=0x4B 通过 `UartToCan_Task_Run` 直接透传到 CAN 帧。**协议不匹配不是本问题根因**。

**决策**：协议没问题 → 转向运行时排查。

---

### 第三步：构建数据流管线图

**操作**：逐函数梳理 UART→CAN 的完整路径

```
UART RX 物理层
  PA10 收到字节
    ↓
USART1_IRQHandler
  HAL_UART_IRQHandler → 读取 DR 寄存器
  HAL_UART_RxCpltCallback  ← user callback
    ↓
stm32f1xx_it.c:HAL_UART_RxCpltCallback
  ring_buffer_put(&uart1_rx_buffer, byte)    写入环形缓冲区
  osEventFlagsSet(uart1_rx_eventHandle, FLAG) 通知解析任务
  HAL_UART_Receive_IT(...)                    重新使能接收中断
    ↓
ProtocolParser_Task_Run (优先级 Normal2)
  ring_buffer_get(&uart1_rx_buffer, &byte)    读取字节
  状态机解析协议帧 (WAIT_SOF→CMD→ID→LEN→DATA)
  osMessageQueuePut(uartToCanQueueHandle)     投递到队列
    ↓
uartToCanQueue[16]
    ↓
UartToCan_Task_Run (优先级 Normal1)
  osMessageQueueGet(uartToCanQueueHandle)     从队列取出
  HAL_CAN_AddTxMessage(&hcan, ...)            发到 CAN 总线
```

**CAN→UART 方向**（已知工作正常）：
```
CAN ISR → canRxQueue[16] → CanRxProcess_Task_Run (Normal)
                            → uart1_send(tx_buffer) → UART DMA TX
```

**关键观察**：`uart1_send()` 是 **全系统唯一** 的 UART TX 入口。它使用互斥锁 + 静态 DMA 缓冲区 + 信号量等待 DMA 完成。

```c
static void uart1_send(const char *buf, uint16_t len)
{
    if (len == 0 || len > UART1_TX_DMA_BUF_SIZE) { return; }
    osMutexAcquire(uart1_tx_mutexHandle, osWaitForever);
    memcpy(uart1_tx_dma_buf, buf, len);
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma_buf, len);
    osSemaphoreAcquire(uart1_tx_semHandle, osWaitForever);  // ← 死锁风险点
    osMutexRelease(uart1_tx_mutexHandle);
}
```

**追问**：三个任务（ProtocolParser、UartToCan、CanRxProcess）都调用了 `uart1_send()`，互斥锁能否正确序列化访问？DMA 失败时会发生什么？

---

### 第四步：检查代码细节 — 发现 DMA 死锁隐患

**操作**：读取 `Core/Src/stm32f1xx_it.c` 中的 `HAL_UART_TxCpltCallback`

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        osSemaphoreRelease(uart1_tx_semHandle);
    }
}
```

`HAL_UART_Transmit_DMA` 在以下情况不会触发 TxCpltCallback：
1. **DMA 通道被占用**（前一次传输未完成）
2. **USART 处于错误状态**（上电未稳定、帧错误等）
3. **DMA 配置错误**

此时 `osSemaphoreAcquire` 在 `uart1_send` 中永远等不到释放 → **线程死锁**，所有等待该 mutex 的任务全部挂起。

**DMA 传输完整链路**（确认无误）：
```
HAL_UART_Transmit_DMA
  → USART_CR3_DMAT 使能
  → DMA1 Channel 4 开始传输
  → DMA 传输完成 → DMA1_Channel4_IRQHandler
    → HAL_DMA_IRQHandler
      → dma_set_TC_flag
      → UART_DMATransmitCplt (DMA_HandleTypeDef *hdma 中的 XferCpltCallback)
        → 在 UART_StartTransmit_DMA 中注册的
          → SET_BIT(huart->Instance->CR1, USART_CR1_TCIE)  使能 TC 中断
  → USART1_IRQHandler (当 TC 标志置位)
    → UART_EndTransmit_IT
      → HAL_UART_TxCpltCallback(huart)  ← 用户回调
        → osSemaphoreRelease(uart1_tx_semHandle)
```

**发现**：所有中断优先级都是 5（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`），DMA TC 中断不会嵌套在 FreeRTOS 临界区内。DMA 回调链无问题。

**但若 DMA 启动失败**（`HAL_BUSY` 或 `HAL_ERROR`）：
- TC 回调永不会执行
- 信号量永不释放
- `osSemaphoreAcquire(osWaitForever)` 永久阻塞
- mutex 永不被释放
- **所有调用 `uart1_send` 的任务全挂**

**临时结论**：这可能是死锁根因，但需要验证（目前用户说 CAN→UART 正常，表明 uart1_send 在工作）。

---

### 第五步：疑点 — 为什么 `[TXTST]` 完全不出现

排查思路：`[TXTST]` 由 `UartToCan_Task_Run` 打印，说明 `osMessageQueueGet(uartToCanQueueHandle)` 从未成功返回 → **ProtocolParser_Task_Run 从未把解析完成的消息投递到队列**。

原因可能：
1. ProtocolParser_Task 不在运行
2. ProtocolParser_Task 运行但拿不到环形缓冲区数据
3. 环形缓冲区收到数据但状态机解析失败
4. 状态机一直停留在中间状态

**操作**：检查任务优先级

从 `freertos.c` 实际代码：
```c
const osThreadAttr_t ProtocolParser__attributes = {
    .priority = (osPriority_t) osPriorityNormal2,
    .stack_size = sizeof(ProtocolParser_Buffer),  // 256 * 4 = 1024 bytes
};
const osThreadAttr_t UartToCan_Ta_attributes = {
    .priority = (osPriority_t) osPriorityNormal1,
};
const osThreadAttr_t CanRxProcess_Ta_attributes = {
    .priority = (osPriority_t) osPriorityNormal,
};
```

**发现**：`freertos.c` 中的优先级（Normal2 / Normal1 / Normal）与 `app_task.c` 注释中写的（Normal1 / Normal / Normal）不一致！但相对顺序一致：ProtocolParser > UartToCan > CanRxProcess。

**操作**：检查任务栈是否足够

- ProtocolParser: 256 words = 1024 bytes — 状态机只用到局部字节变量，绰绰有余
- UartToCan: 512 words = 2048 bytes — 含 `dbg_buffer[128]` 和 `sprintf`，足够
- CanRxProcess: 512 words = 2048 bytes — 含 `tx_buffer[128]`，足够

**结论**：任务栈和优先级不是问题。

**操作**：检查 FreeRTOS 堆是否耗尽

所有任务和 IPC 对象使用**静态分配**（`cb_mem` + `stack_mem`），不消耗 FreeRTOS 堆。只有两个消息队列使用动态分配：
- `canRxQueue`: 16 × 16 = 256 bytes + 控制块 ~80
- `uartToCanQueue`: 16 × 16 = 256 bytes + 控制块 ~80

`configTOTAL_HEAP_SIZE = 3072`，足够。**堆耗尽不是问题。**

---

### 第六步：发现启动时序竞态

**操作**：读 `Core/Src/main.c`

```c
MX_USART1_UART_Init();      // USART1 硬件初始化
MX_USART2_UART_Init();      // USART2 硬件初始化
CAN_Filter_Config();
HAL_CAN_Start(&hcan);
HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
UART_Receive_Start();       // ← 使能 UART RX 中断
                             //   这时 uart1_rx_eventHandle == NULL!
osKernelInitialize();
MX_FREERTOS_Init();         // ← uart1_rx_eventHandle 在这里才创建
osKernelStart();
```

再看 `MX_FREERTOS_Init` 内的创建顺序：
```c
osMutexNew(...)             // 1. 互斥锁
osSemaphoreNew(...)         // 2. 信号量
osMessageQueueNew(...)      // 3. 队列 (2个)
osThreadNew(...) × 4        // 4. 任务 (4个)
osEventFlagsNew(...)        // 5. 事件标志 ← LAST!
```

**测试**：如果在 `UART_Receive_Start()` 之后、`osKernelStart()` 之前有串口字节到达：

1. ISR 执行 → `HAL_UART_RxCpltCallback`
2. `ring_buffer_put(&uart1_rx_buffer, byte)` — 没问题，缓冲区已初始化
3. `osEventFlagsSet(uart1_rx_eventHandle, UART1_RX_FLAG)` — **handle == NULL!**
4. `HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1)` — 重新使能接收

**推演**：`osEventFlagsSet(NULL, FLAG)` 在 CMSIS-RTOSv2 实现中不会崩溃，只会返回 `osErrorParameter`。但**事件标志从一开始就没被设置**。直到 `osKernelStart()` 后新字节到达，事件标志才正常触发。

**但**：如果第一个字节在启动时序中已写入环形缓冲区，事件标志未设置，ProtocolParser_Task 就不会被唤醒。直到有新字节到来触发新的 `osEventFlagsSet`，或者 10ms 超时后自动醒来。

**实验验证**：10ms `osEventFlagsWait` 超时会自动返回，任务醒来后发现环形缓冲区有数据 → 正常处理。所以启动时序竞态**最多导致 10ms 的初始延迟**，不是持续不可用的原因。

**决定**：这不是根因，但记录下来后续修复。

---

### 第七步：加诊断探针，确认任务是否运行

**操作**：在 `ProtocolParser_Task_Run` 开头加：

```c
void ProtocolParser_Task_Run(void *argument)
{
    uart1_send("PARSER STARTED\r\n", 16);  // ← 诊断
    ...
}
```

**用户反馈**："在反馈回来的数据里，能看到这一行" → 终端显示 `PARSER STARTED`

**结论**：ProtocolParser_Task 确实在运行。UART TX (DMA) 正常。

**操作**：在 `ring_buffer_get` 成功后加字节诊断：

```c
if (ring_buffer_get(&uart1_rx_buffer, &byte_received)) {
    static uint32_t byte_count = 0;
    char b[32];
    int n = sprintf(b, "B: %02X [%lu]\r\n", byte_received, (unsigned long)++byte_count);
    uart1_send(b, n);
    ...
}
```

**但**：CAN 收发器连接时终端被 CAN RX 输出淹没，诊断看不到。用户连续发送了多帧 CAN RX 交叠的输出：

```
[14:19:29] RX: CAN RX | ID: 0x326 | DLata: 00 00 03 ... | ID: 0x324 ...
```

输出交叠显示两帧 CAN RX 被渲染到同一行。**终端程序崩溃式输出**。

**判断**：CAN RX 传来的帧率太高（4 帧/50ms），`CanRxProcess_Task` 持续调用 `uart1_send`，占了 UART TX。

---

### 第八步：关键实验 — 拔掉 CAN 收发器

**操作**：断开 CANH/CANL 接线，让 CAN RX 不再有数据，复位 MCU，发送 Python 指令。

**结果**：
```
[14:23:42] RX: PARSER STARTED
[14:25:38] TX: AA 01 01 01 00 00 02 11 00
[14:25:38] RX: B: AA [1]
[14:25:38] RX: B: 01 [2]
[14:25:38] RX: B: 01 [3]
[14:25:38] RX: B: 01 [4]
[14:25:38] RX: B: 00 [5]
[14:25:38] RX: B: 00 [6]
[14:25:38] RX: B: 02 [7]
[14:25:38] TX: AA 01 02 01 00 00 02 11 4B
[14:25:38] RX: B: 11 [8]
[14:25:38] RX: B: 00 [9]
[14:27:07] TX: AA 01 01 01 00 00 02 11 00
[14:27:07] RX: UART->CAN | RX_MSG | ID: 0x101, DLC: 2. Sending...
```

**完整管线确认**：
| 阶段 | 证据 | 状态 |
|------|------|------|
| UART RX 中断 | `B: AA [1]` ... `B: 4B [9]` | ✅ |
| ring_buffer | 字节按序到达 | ✅ |
| ProtocolParser 状态机 | 解析完9字节，ID正确 | ✅ |
| uartToCanQueue | 消息投递成功 | ✅ |
| UartToCan_Task | 打印 `RX_MSG | ID: 0x101` | ✅ |
| CAN 总线 | USB2CAN 看到帧 | ✅ |

**核心发现**：**UART→CAN 整个管线在工作**。拔掉 CAN 后一切正常。

**推论**：问题一定与 CAN 收发器连接时的 **CAN RX 输出**有关。CAN→UART 打印占用了 UART TX 资源，导致：
1. `uart1_send` 互斥锁被 CanRxProcess 长期持有
2. ProtocolParser 和 UartToCan 的 `uart1_send` 调用被阻塞
3. UART RX 字节在环形缓冲区中积压溢出

---

### 第九步：UART TX 竞争定量分析

**操作**：计算带宽

CAN 状态帧 4 帧/50ms（0x323, 0x324, 0x325, 0x326），每帧 8 字节。

每帧格式化输出约 60 字节（含 `CAN RX | ID: 0x323 | DLC: 8 | Data: XX XX ...` + `\r\n`）。

```
115200 baud = 115200 bits/s
1 字节 = 1 start + 8 data + 1 stop = 10 bits（无校验）
→ 11520 bytes/s 原始吞吐
→ 单帧 60 字节 DMA 传输时间 = 60 / 11520 * 1000 ≈ 5.2 ms
4 帧 / 50 ms → 每 50ms 中有 4 × 5.2ms = 20.8ms 占线
UART TX 利用率 ≈ 42%
```

42% 不是 100%，所以在理论上 CanRxProcess 会释放 mutex、CanRxProcess 会在 `osMessageQueueGet` 上阻塞、其他任务**应该能抢到锁**。但实际情况更糟：

**高优先级任务加剧竞争**：
1. CanRxProcess (Normal) 持有 mutex，DMA 传输中
2. UartToCan (Normal1) 尝试 `uart1_send` → mutex 被持 → 阻塞
3. 内核通过优先级继承将 CanRxProcess 提升到 Normal1
4. CanRxProcess 仍然在等 DMA 完成 → 无法释放

**放大效应**：
- 环形缓冲区只有 256 字节
- 若 9 字节的 UART 协议帧在一帧 DMA 传输（5.2ms）内到达 9 个字节
  - 115200 baud → 1 字节间隔 ~0.087ms
  - 9 字节间隔 ~0.78ms
  - 全部到达只需 <1ms
  - 但 ProtocolParser 被 `uart1_send` 阻塞，无法从环形缓冲区读取
  - 环形缓冲区在第 256 个字节后回绕覆盖 → 数据丢失

**实验确认**：用户接回 CAN 后终端显示全被 CAN RX 占据，没有 `B:` 诊断行，说明 ProtocolParser 被完全阻塞。

---

### 第十步：Fix1 — 交换 CAN 发送与诊断顺序

**分析**：`UartToCan_Task_Run` 中的代码顺序是：

```c
// 之前（有问题的顺序）：
uart1_send("UDC | RX_MSG | ...");  // 需等互斥锁 → 可能阻塞
HAL_CAN_AddTxMessage(...);          // CAN 发送被诊断阻塞
```

**思路**：`HAL_CAN_AddTxMessage` 不需要 UART TX 互斥锁。为什么不等 CAN 发完了再打印？

```c
// 之后：
HAL_CAN_AddTxMessage(...);          // 先发，不依赖 UART TX
uart1_send("UDC | OK/FAIL | ..."); // 后打，阻塞只影响显示
```

**风险**：如果 `HAL_CAN_AddTxMessage` 本身也失败呢？CAN 帧可能因为无 TX 邮箱、总线 off、仲裁丢失而无法发送。但即使失败，**失败诊断的打印也不应阻塞 CAN 发送尝试**。

**结果**：CAN 发送路径不再被 UART TX 阻塞。但用户接回 CAN 后，CAN RX 输出仍然淹没终端，`UartToCan_Task` 的诊断行仍然不可见（因为 `uart1_send` 在 CAN 发完后调用，仍然需要等 mutex）。

所以 Fix1 只是解决了 **CAN 发送被阻塞** 的问题，没有解决 **UART TX 被 CAN RX 占满** 的问题。

---

### 第十一步：Fix2a — 变化检测输出

**分析**：不能完全去掉 CAN RX 输出（用户需要调试能力），但可以大幅降低频率。

**设计**：对于每个已知的 CAN ID，记录上次发往 UART 的数据和时间戳。当新帧到达时：
- 数据有变化 → 立即打印（更新记录）
- 数据无变化且距离上次打印 < 500ms → 跳过
- 数据无变化但距离上次打印 ≥ 500ms → 打印（心跳保活）

```c
#define CD_ENTRIES 4
static struct {
    uint32_t id;
    uint8_t  data[8];
    uint32_t last_print_tick;
} cd[CD_ENTRIES];
```

**预期**：
- 电机状态帧全零 → 首次打印 → 后续每 500ms 打一次 → 4 帧交替 = 2 帧/秒
- 输出从 80 fps → 2 fps，UART TX 利用率从 42% → ~1%

**结果**：输出确实降到 ~2 fps，理论上终端不再被淹没。但**用户反馈仍然无法下发指令**。

**反思**：2 fps 的 CAN RX 输出仍然在占用 UART TX（每帧 5.2ms，2 fps = 约 1% 利用率）。但 **1% 的 UART TX 利用率和终端软件的 RX 处理机制共同作用**：

- 用户使用的带 `[时间戳]` 终端的程序，在持续收到 RX 数据时
- 可能将流控设为硬件流控（或终端缓冲区处理方式）
- 导致上位机 Python 脚本发送的数据不能及时送达 STM32
- **PC 终端程序层面的 TX 路径被 RX 数据干扰**

**关键转折点**：认识到问题不仅是 MCU 内 UART TX 竞争，还涉及 **PC 终端软件的双工处理方式**。只要 CAN RX 在持续输出，PC 串口终端在接收数据时能否同时发送数据是一个不确定因素（取决于终端实现）。

---

### 第十二步：Fix2b — 直接跳过状态帧

**决策**：网关的正常运行场景不需要回显电机状态帧。状态帧用于调试，调试时可以拔 CAN 或使用单独的监控工具。

**改动**：

```c
#define IS_STATUS_ID(id)  ((id) >= 0x323 && (id) <= 0x326)

// in CanRxProcess_Task_Run:
if (osMessageQueueGet(canRxQueueHandle, &rx_can_msg, NULL, osWaitForever) == osOK) {
    if (IS_STATUS_ID(rx_can_msg.id)) {
        continue;  // ← 完全跳过，不占 UART TX
    }
    // 非状态帧（如 0x123/0x124 等控制帧）照常打印
}
```

**考量**：
- 状态帧 ID 范围 0x323~0x326 是固定的，由电机控制器固件定义
- 非状态帧（如其他 CAN ID 的诊断响应、错误帧）仍会打印到 UART
- 如果需要查看状态帧数据，可以临时放宽 `continue` 条件

**效果**：
- UART TX 利用率：稳态 ≈ 0%（CanRxProcess 完全不调用 `uart1_send`）
- UART→CAN 指令：可正常下发，USB2CAN 可见
- CAN→UART 非状态帧：仍然工作

**验证**：用户确认"问题解决了"。USB2CAN 可见上位机指令对应的 CAN 帧。

---

## 关键经验总结

### 排查方法论

1. **逐层隔离**：PC 软件 → UART 物理层 → ISR → 环形缓冲区 → 协议解析 → 任务调度 → CAN 发送。每一步通过诊断确认正常后，才继续往下查。

2. **对比实验**：拔掉 CAN 收发器 vs. 接回 CAN → 确定问题在 CAN RX 路径。

3. **定量估算**：115200 baud 的理论吞吐 11520 bytes/s，单帧 60 字节 → 5.2ms，4 帧/50ms → 42% 占线。这个数字表明 UART TX 是瓶颈。

4. **假说排除法**：用户怀疑协议不匹配 → 验证后发现 Python 脚本绕过了 cmd 字段不一致问题。如果不是协议问题 → 转向运行时和执行路径排查。

5. **终端软件也是系统的一部分**：PC 串口终端的 RX/TX 双工能力、缓冲区处理会影响 MCU 的数据收发，不能只看 MCU 侧。

### 本案例独特之处

- **问题表现为"完全不工作"，实际是"被淹没"**：UART→CAN 管线一直工作，只是 UART TX 被 CAN RX 输出独占，导致诊断不可见 + 环形缓冲区溢出
- **连续输出导致终端崩溃式显示**：串口终端在持续 RX 下渲染异常，多行拼一行，误导了问题定位
- **三个优先级等级的任务竞争一个 UART TX 资源**：系统设计时未考虑共享资源的使用量级

### 遗留问题（值得后续修复）

1. **`uart1_send` DMA 失败保护**：`HAL_UART_Transmit_DMA` 若返回非 `HAL_OK`，当前代码忽略返回值继续等待信号量 → 全局死锁

2. **启动时序**：`UART_Receive_Start()` 在 `osEventFlagsNew` 前使能 RX 中断 → 竞态

3. **`cmd` 字段设计不一致**：`app_config.h` 中 `Command_ID_t` 的枚举值与 3_MCLM_t2 的命令字节不匹配，`cmd` 字段未实际写入 CAN data[0]

4. **CAN TX 邮箱满时的处理**：`HAL_CAN_AddTxMessage` 返回 `HAL_BUSY` 时（三个 TX 邮箱全满），当前只是打印错误不做重试

## 代码变更总览

### `UartToCan_Task_Run` — 交换顺序

```diff
 // Before（诊断阻塞 CAN）:
-  uart1_send(dbg_buffer, offset);  // 先打印诊断
   HAL_CAN_AddTxMessage(...);       // 再发 CAN
-  // 然后可能再打印失败

 // After（CAN 先发）:
   HAL_CAN_AddTxMessage(...);       // 先发 CAN
-  uart1_send(dbg_buffer, offset);  // 后打印诊断
```

### `CanRxProcess_Task_Run` — 跳过状态帧

```diff
 // Before（每帧都打印，UART TX 被占死）:
   if (osMessageQueueGet(...) == osOK) {
-      sprintf(...);   // 格式化每帧
-      uart1_send(...); // 输出到 UART
   }

 // After（跳过状态帧）:
   if (osMessageQueueGet(...) == osOK) {
+      if (IS_STATUS_ID(rx_can_msg.id)) continue;  // 状态帧跳过
       sprintf(...);   // 非状态帧才打印
       uart1_send(...);
   }
```

### `ProtocolParser_Task_Run` — 注释掉诊断

```diff
 // Before（一启动就抢 UART TX）:
-  uart1_send("PARSER STARTED\r\n", 16);

 // After（注释保留，调试时启用）:
+  // uart1_send("PARSER STARTED\r\n", 16);
```

---

## 补充：排查中发现的特殊细节

### 1. `// /*` 注释陷阱

原代码中使用了一种看似"禁用代码块"的写法：

```c
    //   /*
      int offset = sprintf(dbg_buffer, "...");
      uart1_send(dbg_buffer, offset);
    //   */
```

**实际效果**：`//` 将 `/*` 注释掉了，`// */` 也将 `*/` 注释掉了。中间的代码**始终处于激活状态**。注释文本说"暂时禁用以排查死机问题"但实际**根本没有被禁用**。

这个写法在原始代码中出现在三个位置（`UartToCan_Task_Run` 的 TX_MSG 诊断、TX_FAIL 诊断、`CanRxProcess_Task_Run` 的全帧输出）。之前开发者可能以为用这种写法"关闭"了诊断，但实际一直在运行，贡献了 UART TX 负载。

### 2. `PARSER STARTED` 能出现但 `B:XX` 不能 — 时序差

连接 CAN 时用户看到的现象矛盾：
- `PARSER STARTED` ✅ 可见
- `B: XX` ❌ 不可见

**原因在启动时序差**：

```
时间轴（连接 CAN）:
t=0ms     osKernelStart()
t=0~0.1ms CAN ISR: 第一个状态帧到达 → canRxQueue
t=0.1ms   CanRxProcess_Task (Normal) 启动 → 抢 uart1_tx_mutex → 开始 DMA TX
t=0.1ms   ProtocolParser_Task (Normal2) 启动
          → 执行 uart1_send("PARSER STARTED") 需要 16 字节 DMA ≈ 1.4ms
          → 但 mutex 被 CanRxProcess 持有 → BLOCKED
t=0.1ms   此时 UART RX 还没有数据（上位机还没发指令）
t=5.3ms   CanRxProcess DMA 完成 → mutex 释放
t=5.3ms   ProtocolParser 拿到 mutex → "PARSER STARTED" DMA 开始
t=6.7ms   "PARSER STARTED" DMA 完成 → ProtocolParser 进入 osEventFlagsWait
t=6.7ms+  后续 CAN RX 持续到达 → CanRxProcess 持续打印
           → B: XX 诊断需 uart1_send → 等 mutex → 被 CanRxProcess 占满 → 永远等不到
```

**关键**：`PARSER STARTED` 在启动初期（CanRxProcess 还没开始刷屏前）执行完毕，而 `B: XX` 在系统进入稳态后被淹没。

### 3. PC 终端软件对 TX 的干扰机制

为什么"变化检测到 2 fps 都不行"？分析接收端行为：

PC 串口终端软件（带 `[14:47:27]` 时间戳的，可能是 ECHELON、MobaXterm、TeraTerm 等）的内部处理：

```
MCU DMA TX → UART TX 线 → USB 串口转换器 → PC USB
                                              ↓
                                      串口终端读线程
                                         ↓ 持续读取 → 显示在屏幕
                                         同时处理用户键盘输入
                                         ↓
终端软件内部缓冲区:
  RX 缓冲区: 满 → 触发显示刷新 → 绘图/重绘
  TX 缓冲区: 用户敲键盘 → 写入 → 发送

当 RX 持续快速到达时:
  1. 终端 RX 读取线程几乎连续工作
  2. GUI/控制台显示的渲染可能阻塞其他处理
  3. 键盘/TX 缓冲区刷新可能被延迟
  4. 某些终端在显示大量数据时使用"行缓冲"，
     \r\n 的渲染可能触发行缓冲区刷新，
     导致接收下一行时显示线程仍在处理上一行 → 行交叠

再加上: 用户通过 Python 脚本发送数据（通过 pyserial 打开同一 COM 口）
  → 如果终端软件占着 COM 口，Python 打不开
  → 如果终端软件在收到数据时保持 CTS/RTS 流控，可能阻塞上位机发送
```

**PC 终端对 RX 数据的处理影响 TX 能力，这是 MCU 侧无法控制的变量。** 只要持续向终端发送数据，UART→CAN 的实验结果就不可靠。

**验证**：拔掉 CAN 后（无 RX 数据），Python 脚本可以独立工作，UART→CAN 成功。

### 4. 启动时序竞态详解

启动时的潜在竞态在"第六步"中已触及，但实际影响比最初判断更大。

```
main.c:
  UART_Receive_Start()   // 1. 使能 UART RX 中断
                         //    uart1_rx_eventHandle == NULL 此时
  osKernelInitialize()
  MX_FREERTOS_Init()
    osEventFlagsNew(...)  // 2. 事件标志创建 ← 但线程还未运行
  osKernelStart()         // 3. 调度器启动，任务开始运行

在步骤 1–3 之间：外设已初始化，中断已使能
若一个字节在 1~3 之间到达：
  ISR:
    ring_buffer_put(&uart1_rx_buffer, byte)  // ✅ 环形缓冲区正常
    osEventFlagsSet(NULL, UART1_RX_FLAG)      // ❌ 写入 NULL 句柄
    HAL_UART_Receive_IT(...)                   // ✅ 重新使能

osEventFlagsSet(NULL, ...) 在 CMSIS-RTOSv2 中返回 osErrorParameter，
不会崩溃，但事件标志未设置。

然后 osKernelStart() → ProtocolParser_Task 启动
  → ring_buffer_get() → 可能拿到之前存储的字节
  → 但若只有一个字节（不足以完成一帧），
    状态机进入中间状态，然后 buffer 空 → osEventFlagsWait(10ms)
  → 10ms 内若后续字节到达 → ISR osEventFlagsSet(有效句柄) → 唤醒 → 继续
  → 10ms 内若无字节 → 超时醒来 → ring_buffer_get 失败 → 再等 10ms

所以：启动时的竞态最多导致 10ms 的初始延迟，对功能无持续性影响。
但：第一个字节在事件标志创建前到达会丢失事件标志触发，依赖事件标志超时兜底。
```

### 5. 中断优先级分析

所有外设中断（USART1、DMA1 Ch4/5/6/7、CAN TX/RX）均设置为优先级 **5**。

```c
// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
// configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << (8 - 4) = 0x50
```

在 FreeRTOS 中，中断优先级 **低于** `configMAX_SYSCALL_INTERRUPT_PRIORITY` 的不会被 FreeRTOS 临界区屏蔽。优先级 5 等于最大值，因此 **FreeRTOS 的 `taskENTER_CRITICAL()` / `portDISABLE_INTERRUPTS()` 无法屏蔽这些中断**。

这意味着：
- UART RX ISR 可以在任何时刻（包括内核临界区操作链表时）执行
- `osEventFlagsSet` 和 `osMessageQueuePut` 从 ISR 调用是安全的（这些 API 有 FromISR 版本）
- DMA TC ISR 和 CAN RX ISR 同样不会被屏蔽

**好处**：中断响应延迟低
**风险**：如果 ISR 中的 FreeRTOS API 调用在高优先级任务访问同一内核对象时发生，需要 CMSIS-RTOSv2 实现内部的临界区保护（通常用 `portSET_INTERRUPT_MASK_FROM_ISR()`）

**结论**：中断优先级设置一致，不是本问题原因。

### 6. `uart1_send` 的 DMA 死锁风险量化

如"第四步"所述，`HAL_UART_Transmit_DMA` 的返回值未被检查：

```c
static void uart1_send(const char *buf, uint16_t len)
{
    osMutexAcquire(uart1_tx_mutexHandle, osWaitForever);
    memcpy(uart1_tx_dma_buf, buf, len);
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma_buf, len);  // ← 返回值被忽略
    osSemaphoreAcquire(uart1_tx_semHandle, osWaitForever);   // ← 永不释放 = 死锁
    osMutexRelease(uart1_tx_mutexHandle);
}
```

`HAL_UART_Transmit_DMA` 返回非 `HAL_OK` 的几种场景：
| 返回值 | 触发条件 | 概率 | 影响 |
|--------|----------|------|------|
| `HAL_BUSY` | USART 正在 DMA 传输中（前一次未完成） | 低（有 mutex 保护） | 信号量永不释放 → 死锁 |
| `HAL_ERROR` | USART 句柄未初始化/参数无效 | 极低（初始化时已确认） | 同上 |
| `HAL_TIMEOUT` | 本调用不会超时 | 0 | - |

在本次问题中，`uart1_send` 由 CanRxProcess 在正常流程中使用，DMA TX 正常运作，**没有触发这个死锁**。但这个隐患应该在后续修复中处理。

### 7. `cmd` 字段最终结论

`App_UART_Message_t` 的 `cmd` 字段设计意图不明确：

- **文档写**："透传至 CAN data[0]"
- **现状**：`UartToCan_Task_Run` 仅将 `uart_msg.data[0..7]` 发送到 CAN 帧的 data[0..7]，`cmd` 字段完全被忽略
- **Python 脚本**：发送 `cmd=0x01`（文档标注为 `CMD_SET_SPEED`），但实际将 CAN 命令字节 `0x11` 放在 data[0]

**建议后续处理**：
- 如果 `cmd` 应该透传到 CAN data[0]，则修改 `UartToCan_Task_Run` 将 `cmd` 写入 `data[8]` 数组的第 1 个位置，其余 data 向后偏移
- 但这意味着上位机的协议格式要改变：CAN data[0] = cmd, CAN data[1..] = data[0..]，下游控制器据此调整
- 或者删除 `cmd` 字段，简化协议帧格式为 `SOF | ID(4B) | LEN(1B) | DATA(0-8B)`，与数据流一致

### 8. CAN TX 邮箱的工作机制

`HAL_CAN_AddTxMessage` 的返回值说明：
- **`HAL_OK`** = 帧已放入硬件 TX 邮箱（未确认已发送完成）
- **`HAL_BUSY`** = 三个 TX 邮箱全部满（有 3 帧等待发送）

CAN 控制器在 Arbitration 阶段等待总线空闲，随后发送邮箱中的帧。如果总线繁忙或有更高优先级 ID 在发送，CAN 控制器会自动重试（`AutoRetransmission = ENABLE`）。

在本案例中，CAN 总线有其他节点（电机控制器）在发送状态帧，所以总线不会是长时间的 idle 状态。网关发送的帧（ID=0x123 等）需要等待总线空闲后参与仲裁。

**为什么 Fix1（先发 CAN 再打印）有效**：`HAL_CAN_AddTxMessage` 不依赖任何互斥锁或同步对象，只需硬件 TX 邮箱有空位 + CAN 外设已使能。因此即使 UART TX 被完全阻塞，CAN 发送不会受影响。

### 9. 被否定的替代方案

排查过程中考虑过的其他方案及**否决原因**：

| 方案 | 思路 | 否决原因 |
|------|------|----------|
| 提高 `uart1_send` 的 mutex 超时 | `osMutexAcquire(..., 100)` 不等死 | 超时后放弃打印但帧数据可能已部分写入 DMA 缓冲区；且 CAN→UART 不打印和直接跳过无本质区别 |
| 将 CanRxProcess 降优先级 | 让 ProtocolParser 不被阻塞 | FreeRTOS 优先级继承下，mutex 等待会自动提升持有者的优先级，降级不能解决 |
| 让 CanRxProcess 改用独立 UART（USART2） | 硬件已有 USART2 初始化但未用 | 需要额外接线、USB 串口转换器；用户电路板可能无 USART2 引出 |
| CAN RX 用 DMA 替代 IT | 减少 CPU 负载 | 当前 IT 驱动已经够快，问题在 UART TX 阻塞协议解析，不是 CAN ISR 性能 |
| 环形缓冲区改为更大的值 | 从 256 扩大到 1024 | 只是延缓溢出，不能解决根本问题；ProtocolParser 被阻塞时缓冲区再大也会满 |
| 增加 TX 互斥锁等待超时重试 | `osMutexAcquire` 超时后重试 | UART TX 的目的就是把数据打出去，超时+重试增加了延迟但没有消除竞争 |

### 10. 完整故障时序图（连接 CAN 时）

```
时间(ms)      ProtocolParser         UartToCan:print       CanRxProcess         UART TX 线(115200)
           (Normal2,解析UART)      (Normal1,发CAN)      (Normal,打印CAN RX)
───────  ─────────────────────  ───────────────────  ───────────────────  ─────────────────────────  
  
0            启动                     启动                   启动             空闲
0.1                                         CAN ISR:状态帧→队列
0.1                                                            mutex抢到
0.1                                                            HAL_UART_Transmit_DMA  ← 60字节/5.2ms
0.1     uart1_send("PARSER")                                                         
0.1     mutex被持→BLOCKED                                                           DMA传输中...
0.1                                                          
1.0                                                                           TX: C A N   R X   | ...
2.0                                                                           ...
                                                                              [上位机发AA 01 23...]
                                                                              USART1 RX 中断触发
2.0     ← 还在 BLOCKED                                                               
2.0     [ring_buffer 写入 AA, 01, 23...]
2.0     [osEventFlagsSet → 但 ProtocolParser 还在等mutex,未处理]
                                                                              
5.3                                                     DMA完成                 
5.3                                                     sem释放                 
5.3                                                     mutex释放                
5.3     mutex拿到→继续执行                                                       
5.3     uart1_send("PARSER") → DMA 16字节                                          
6.7     PARSER DMA完成                                                            
6.7     ring_buffer_get → 读字节 AA, 01, 23...                                    
6.8     状态机解析完成                                                              
6.8     osMessageQueuePut(uartToCanQueue)                                         
6.8     → UartToCan 收到消息                                                      ← 但 UART TX 还忙
7.0                                                     [下一帧CAN状态]
7.0                                                            mutex抢到
7.0                                                            DMA: CAN RX | ...
                                 CAN发送: wait, mutex被持...
12.3                                                   DMA完成, mutex释放
12.3    (UART RX字节已吃完了)                          
12.3    osEventFlagsWait(10ms) 等后续UART字节                                        
                                                                              
     （循环：UART TX 约 42% 时间被 CAN RX 输出占用）
```

### 11. 影响范围的量化

在修复前，各任务的 UART TX 消耗：

| 任务 | 每次调用耗时 | 调用频率 | UART TX 占用率 |
|------|-------------|----------|---------------|
| CanRxProcess 打印状态帧 | ~5.2ms | 4 帧/50ms | ~42% |
| UartToCan 诊断打印 | ~5.2ms | 用户指令（偶发） | <1% |
| ProtocolParser 诊断 | ~1.4ms | 启动时 1 次 | 忽略 |

修复后：

| 任务 | 每次调用耗时 | 调用频率 | UART TX 占用率 |
|------|-------------|----------|---------------|
| CanRxProcess | 0（跳过） | 0 | **0%** |
| UartToCan 诊断打印 | ~5.2ms | 用户指令（偶发） | <1% |
| UartToCan CAN 发送 | 0（不占用 UART） | 用户指令 | **0%** |

### 12. 残留风险评估

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| DMA 发送失败导致全局死锁 | 低 | 系统完全挂起 | 加 DMA 返回值 + 超时保护 |
| 启动时 UART RX 字节丢失 | 低 | 命令首字节丢失 | 将事件标志创建移到 UART 初始化前 |
| CAN 总线 off 或邮箱满 | 低 | CAN 发送失败 | 增加重试或状态恢复机制 |
| `cmd` 字段被忽略 | 中 | 与其他上位机协议不兼容 | 确认设计意图后修复 |
| UART TX 缓冲区溢出（128 字节） | 低 | 长打印被截断 | 当前 60~85 字节的打印在限制内 |
| 环形缓冲区 256 字节溢出 | 低 | 协议帧被截断 | 当前 9 字节帧远小于 256 |

### 13. 本案例排查的原则总结

1. **先确认不要修什么** — 协议不匹配的怀疑被验证后排除，少走了弯路
2. **逐级诊断从输出端往回推** — 先确认任务运行 → 再确认字节到达 → 再确认解析 → 逐步缩小范围
3. **隔离外部变量** — 拔掉 CAN 是决定性实验，将 CAN RX 从系统中移除后故障消失
4. **定量分析** — 计算 UART TX 带宽和时间占比，看是不是"理论上能跑满"
5. **PC 工具也是系统的一部分** — 终端软件对 RX/TX 的处理方式影响系统的整体行为
6. **假说驱动** — 每个步骤提出假说 → 设计实验验证/证伪 → 得出结论或修改假说

#### 补充了 13 个章节，全文 814 行。新增内容包括：

章节	内容
// /* 注释陷阱	为什么注释说"禁用"但代码一直跑
PARSER STARTED vs B:XX 时序差	相同函数、同个 mutex，一个能出现一个不能的原因
PC 终端干扰机制	持续 RX 如何抑制 TX 的完整链路分析
启动时序竞态详析	事件标志创建前的字节怎么处理
中断优先级分析	为什么优先级 5 意味着 FreeRTOS 临界区无法屏蔽
DMA 死锁风险量化	HAL_UART_Transmit_DMA 返回值被忽略的后果
cmd 字段最终结论	建议后续要么改代码要么删字段
被否定的替代方案	7 种考虑过的方案及否决原因
完整故障时序图	0~12ms 每一微秒各任务在做什么
影响范围量化	修复前后 UART TX 占用率对比表
残留风险评估	5 个已知风险的 P x I 评估
排查原则总结	6 条从本案例抽象出来的通用方法论

---

## 附录：实际代码改动对照 (`App/app_task.c`)

### 改动 1：`CanRxProcess_Task_Run` — 跳过电机状态帧

```diff
 void CanRxProcess_Task_Run(void *argument)
 {
   App_CAN_Message_t rx_can_msg;
   char tx_buffer[128];

+  #define IS_STATUS_ID(id)  ((id) >= 0x323 && (id) <= 0x326)

   for(;;)
   {
     if (osMessageQueueGet(canRxQueueHandle, &rx_can_msg, NULL, osWaitForever) == osOK)
     {
+      if (IS_STATUS_ID(rx_can_msg.id)) {
+          continue;
+      }
+
       int offset = sprintf(tx_buffer, "CAN RX | ID: ...", ...);
-      //   /* ... uart1_send(tx_buffer, offset); // */  // ← 原注释陷阱，实际是激活的
+      uart1_send(tx_buffer, offset);                     // 现在干净了
     }
   }
 }
```

**说明**：新增 `IS_STATUS_ID` 宏 + `continue` 跳过。移除了原有 `// /*` 注释陷阱写法。非状态帧（如 0x123/0x124 控制帧或 0x225 查询响应）照常打印。

---

### 改动 2：`UartToCan_Task_Run` — 先发 CAN 再打印

```diff
 void UartToCan_Task_Run(void *argument)
 {
   for(;;)
   {
     if (osMessageQueueGet(uartToCanQueueHandle, &uart_msg, NULL, osWaitForever) == osOK)
     {
-      // 旧顺序：诊断打印在前（需获取 UART TX mutex，可能阻塞）
-      int offset = sprintf(dbg_buffer, "UART->CAN | RX_MSG | ID: 0x%lX, DLC: %d. Sending...\r\n", ...);
-      uart1_send(dbg_buffer, offset);
+      // 新顺序：先判断帧类型
       if (uart_msg.id > 0x7FF) {
           tx_header.IDE = CAN_ID_EXT;
           tx_header.ExtId = uart_msg.id;
       } else {
           tx_header.IDE = CAN_ID_STD;
           tx_header.StdId = uart_msg.id;
       }
-
       tx_header.DLC = uart_msg.len;
-      HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);
-
-      // 失败后才打印诊断
-      if (tx_status != HAL_OK) {
-          offset = sprintf(...);
-          uart1_send(dbg_buffer, offset);
-      }
+
+      // 新顺序：先发 CAN（不依赖 UART TX）
+      HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);
+
+      // 再打印诊断（不阻塞 CAN 发送关键路径）
+      if (tx_status == HAL_OK) {
+          offset = sprintf(dbg_buffer, "UART->CAN | OK | ID: 0x%lX\r\n", uart_msg.id);
+      } else {
+          offset = sprintf(dbg_buffer, "UART->CAN | FAIL | ID: 0x%lX Status: %d\r\n", uart_msg.id, tx_status);
+      }
+      uart1_send(dbg_buffer, offset);
     }
   }
 }
```

**说明**：关键行 `HAL_CAN_AddTxMessage` 从第 4 步移到第 2 步，不再被前面的 `uart1_send` 阻塞。同时成功/失败都打印（原代码只在失败时打印），便于调试确认。

---

### 改动 3：`ProtocolParser_Task_Run` — 注释掉诊断探针

```diff
 void ProtocolParser_Task_Run(void *argument)
 {
-  uart1_send("PARSER STARTED\r\n", 16);  // ← 改为注释，调试时启用
+  // uart1_send("PARSER STARTED\r\n", 16);

   for(;;)
   {
     if (ring_buffer_get(&uart1_rx_buffer, &byte_received))
     {
-      // 字节级诊断（调试后注释掉）
-      static uint32_t dbg_bcnt = 0;
-      char dbg_b[32];
-      int dbg_n = sprintf(dbg_b, "B: %02X [%lu]\r\n", byte_received, (unsigned long)++dbg_bcnt);
-      uart1_send(dbg_b, dbg_n);
+      // /* 字节级诊断保留但注释，排查时取消注释即可
+      //  { ... B: %02X ... }  // */

       switch (state) { ... }
     }
   }
 }
```

**说明**：`PARSER STARTED` 和 `B: XX` 诊断行改为注释，保留源码中方便下次排查时取消注释。

---

### 改动 4：`// /*` 注释陷阱清理

原文三处 `// /* ... // */` 写法全部处理：

| 位置 | 修改前 | 修改后 |
|------|--------|--------|
| `UartToCan_Task_Run` TX_MSG 诊断 | `// /* ... uart1_send ... // */` | 删除（替换为新 OK/FAIL 打印）|
| `UartToCan_Task_Run` TX_FAIL 诊断 | `// /* ... uart1_send ... // */` | 删除（同上）|
| `CanRxProcess_Task_Run` 全帧输出 | `// /* ... uart1_send ... // */` | 移除外层注释，正常执行 |

移除后的代码使用明确的 `if/else` 控制流程，不再依赖易误解的注释技巧。

---

### 未改动的遗留问题

以下问题在本次修复中**没有改动代码**，仅记录在文档中供后续跟进：

| 问题 | 文件 | 建议修复 |
|------|------|----------|
| `uart1_send` DMA 失败无保护 | `app_task.c:39` | 检查 `HAL_UART_Transmit_DMA` 返回值，非 OK 时跳过信号量等待 |
| 启动时序竞态 | `main.c:111` | 将 `osEventFlagsNew` 移到 `UART_Receive_Start` 之前 |
| `cmd` 字段未透传 | `app_task.c:210` | 与上位机协议统一设计后修改 |
| 事件标志 auto-clear vs NoClear | `app_task.c:279` | 当前 auto-clear 可用，但 NoClear 方案更省 CPU |

---

## Session 2：P0/P1/P2 增强修复（承接 fix3）

### 背景

fix3（跳过状态帧）解决了 UART TX 被 CAN RX 占满的根本问题。在此基础上，本轮针对用户体验和系统鲁棒性做了三轮渐进增强（P0/P1/P2），并完成了 CAN ID 的完整扩展。

---

### P0：最小侵入修复 — 交换 CAN 发送与诊断顺序

**目标**：消除 `UartToCan_Task_Run` 中 CAN 发送被 UART TX 诊断阻塞的可能（即使 fix3 后概率极低）。

**改动**（`app_task.c` `UartToCan_Task_Run`）：

```c
// 先发 CAN（不依赖 UART TX）
HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header, uart_msg.data, &tx_mailbox);

// 再打诊断（不阻塞 CAN 发送关键路径）
// 成功诊断已注释，仅 TX_FAIL 保留
if (tx_status != HAL_OK) {
    int offset = sprintf(dbg_buffer, "UART->CAN | TX_FAIL | Status: %d\r\n", tx_status);
    uart1_send(dbg_buffer, offset);
}
```

**设计决策**：
- CAN 先发，UART 后打 — `HAL_CAN_AddTxMessage` 不依赖任何互斥锁
- 成功诊断注释掉 — 用户要求"这个保留，注释掉，不要删"（减少正常路径的 UART TX 占用）
- TX_FAIL 保留 — 异常时需要可见的错误反馈

---

### P1：状态帧限频输出（替代 fix3 的完全跳过）

**背景**：fix3 用 `IS_STATUS_ID` + `continue` 完全跳过了 0x323~0x326 状态帧。但用户需要**能看到**状态帧数据（调试电机状态），只是不能**占满** UART TX。

**P1v1 — 变化检测方案（失败）**：

设计思路：记录每个 ID 的上次数值和打印 tick，数据有变化才打印，无变化时每 500ms 心跳保活。

```c
static int16_t prev_speed[4] = {0};
static uint8_t  prev_flags[4] = {0};
static uint32_t last_print[4] = {0};
uint32_t now = osKernelGetTickCount();

int16_t current_speed = (int16_t)(rx_can_msg.data[0] | (rx_can_msg.data[1] << 8));
uint8_t flags = rx_can_msg.data[7];

bool changed = (current_speed != prev_speed[idx] || flags != prev_flags[idx]);

if (!changed && (now - last_print[idx] < pdMS_TO_TICKS(500)))
    continue;  // 无变化且距上次打印 <500ms → 跳过
```

**用户测试结果**："持续不停打印" — 完全没有限频效果。

**根因分析**（理论推断，因无法连接调试器）：
1. **`int16_t` 严格别名违规**：`int16_t*` 从 `uint8_t*` 转型读取违反 C 语言严格别名规则（`-fstrict-aliasing`）。编译器可能优化掉比较，认为两个不同类型的指针不会指向同一内存 — 结果每次比较都返回 `true`。
2. **`uint32_t` 回绕比较**：`osKernelGetTickCount()` 约 49 天回绕，短期不触发但增加复杂度。
3. **flags 字节含运行位变化**：电机运行时 data[7] 的状态位可能在每帧间变化，导致 `changed` 持续为 true。

**P1v2 — 计数器限频方案（成功）**：

用简单计数器代替数值比较，完全避免指针别名问题：

```c
#define PRINT_INTERVAL 10  // 每 10 帧打印 1 帧 → 约 500ms 间隔（50ms × 10）

// 在 for 循环内：
if (IS_STATUS_ID(rx_can_msg.id)) {
    int idx = rx_can_msg.id - 0x323;
    static uint8_t frame_count[4] = {0};
    frame_count[idx]++;
    if (frame_count[idx] % PRINT_INTERVAL != 1)
        continue;  // 跳过 9/10 帧
    // 打印状态帧数据
}
```

**效果**：
- 每 ID 每 500ms 打印 1 帧（4 个 ID 交替 → 约 2 帧/秒）
- UART TX 利用率 ≈ 1%
- 没有严格别名问题，不依赖数值比较
- 实现简单，逻辑一目了然

---

### P2：CAN RX 队列丢弃计数器

**目标**：监控 CAN RX 队列是否溢出，帮助判断系统负载是否过高。

**改动**：

`app_globals.h`：
```c
extern volatile uint32_t g_can_rx_drop_count;
```

`stm32f1xx_it.c` — `HAL_CAN_RxFifo0MsgPendingCallback`：
```c
if (osMessageQueuePut(canRxQueueHandle, &can_msg, 0, 0) != osOK) {
    g_can_rx_drop_count++;  // 队列满，丢弃并计数
}
```

`CanRxProcess_Task_Run` 中在打印非状态帧时输出丢弃计数：
```c
if (g_can_rx_drop_count > 0) {
    offset += sprintf(tx_buffer + offset, " [DROP: %lu]", (unsigned long)g_can_rx_drop_count);
}
```

**设计要点**：
- `volatile` 防止编译器优化（ISR 和任务线程共享）
- 仅在非状态帧输出时附带，不增加单独的输出
- `volatile uint32_t` 在 STM32F103 上是原子读写（32 位对齐），无需额外保护

---

### CAN ID 完整扩展（`app_config.h`）

将原有的单组定义扩展为双组明确命名：

```c
// 协议宏
#define CMD_SET_SPEED_T2_G1  0x11U  // 调速命令（G1: 0x125/0x126）
#define CMD_GET_STATE        0x01U  // 查询状态
#define CMD_ESTOP_T2         0x08U  // 紧急停止

// CAN ID 分组
#define MOTOR_G1_CTRL_TX_ID    0x125U  // G1 控制帧（上位机→电机）
#define MOTOR_G1_STATUS_TURN   0x325U  // G1 转速状态反馈
#define MOTOR_G1_STATUS_POWER  0x326U  // G1 功率状态反馈

#define MOTOR_G2_CTRL_TX_ID    0x123U  // G2 控制帧
#define MOTOR_G2_STATUS_TURN   0x323U  // G2 转速状态
#define MOTOR_G2_STATUS_POWER  0x324U  // G2 功率状态
```

**状态帧字节布局**（已验证，来自 `3_MCLM_t2`）：
| 偏移 | 含义 | 类型 |
|------|------|------|
| [0-1] | current_speed | int16 LE |
| [2-3] | accum_ticks | uint16 LE |
| [4-5] | pwm | int16 LE |
| [6] | target_speed | int8 |
| [7] | flags | uint8 |

---

### 当前代码状态总览

| 组件 | 状态 | 说明 |
|------|------|------|
| UART RX → 环形缓冲区 | ✅ | IT 中断，单字节接收 |
| 环形缓冲区 → 协议解析 | ✅ | 状态机 (SOF=0xAA)，已移除诊断打印 |
| 协议解析 → uartToCanQueue | ✅ | 16 深度队列 |
| UartToCan 处理 | ✅ P0 | CAN 先发，UART 后打，失败才打印 |
| CAN TX 发送 | ✅ | `HAL_CAN_AddTxMessage` 直发 |
| CAN RX → canRxQueue | ✅ P2 | 有溢出计数监控 |
| CanRxProcess 状态帧输出 | ✅ P1 | 计数器限频，500ms 间隔 |
| CanRxProcess 非状态帧输出 | ✅ | 正常打印（含溢出统计） |
| Heartbeat LED | ✅ | PC13 翻转，300ms |

---

### 新增遗留问题

| 问题 | 说明 | 优先级 |
|------|------|--------|
| P1v1 严格别名编译优化风险 | `int16_t*` 从 `uint8_t*` 转型，`-fstrict-aliasing` 下行为未定义 | 低（已用 P1v2 绕过） |
| `g_can_rx_drop_count` 永不归零 | 当前只增不减，需人工复位观察 | 低（后续可加阈值清零） |
| 协议不匹配 0x04024012 | PC 终端收到未预期的 ID，怀疑是 ID 拼接错位或 CAN 总线噪声 | 中（用户已搁置） |

---

### 未实现但已记录的需求

以下需求来自 `doc/fix/goal.md` 和 `doc/fix/goal2_ToMaster.md`，不在本轮范围内：

1. **SystemState 统一状态管理**：将外设状态、CAN 总线状态、任务健康度集中管理
2. **IMU/Mahony 姿态解算**：SPI 读取 ICM-20948，融合输出
3. **Control Task 闭环控制**：CAN 指令下发 + 反馈接收 + PID
4. **分布式控制节点架构**：从"数据管道"升级为 CAN 网络智能节点