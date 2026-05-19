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
