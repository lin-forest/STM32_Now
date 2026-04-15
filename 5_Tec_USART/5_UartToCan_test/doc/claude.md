# 代码审查与优化总结

## 已完成的全部修改

### P0 — ring_buffer 原子性修复

**文件**: `App/ring_buffer.h`, `App/ring_buffer.c`

**问题**: `head`/`tail` 为 `uint16_t`，ISR 写、任务读，Cortex-M3 不保证原子。

**修改**:
- `head`/`tail` 改为 `volatile uint8_t`
- 缓冲区保持 256 大小，`uint8_t` 自然溢出回绕等价于 `% 256`
- 去掉所有 `% UART_RX_BUFFER_SIZE` 取模运算
- 单字节操作在 ARM Cortex-M3 上天然原子，无需临界区

---

### P0 — UART 多任务互斥

**文件**: `App/app_globals.h`, `App/app_task.c`, `Core/Src/freertos.c`（CubeMX 生成）

**问题**: `UartToCan_Task` 和 `CanRxProcess_Task` 同时直接调用 `HAL_UART_Transmit(&huart1)`，任务切换时输出交错乱码。

**修改**:
- CubeMX 中创建 `uart1_tx_mutex`，生成句柄 `uart1_tx_mutexHandle`
- `app_globals.h` 中声明 `extern osMutexId_t uart1_tx_mutexHandle`
- `app_task.c` 顶部封装 `static void uart1_send()`，内部加锁/解锁
- 两个任务的所有 `HAL_UART_Transmit` 调用统一替换为 `uart1_send()`

---

### P1 — ProtocolParser 事件驱动

**文件**: `Core/Src/stm32f1xx_it.c`, `App/app_task.c`, `App/app_globals.h`, `Core/Src/freertos.c`（CubeMX 生成）

**问题**: 缓冲区空时 `osDelay(1)` 轮询，最坏引入 1ms 延迟，浪费 CPU。

**修改**:
- CubeMX 中创建 `uart1_rx_event`，生成句柄 `uart1_rx_eventHandle`
- `app_globals.h` 中声明句柄及标志位 `#define UART1_RX_FLAG 0x01U`
- `HAL_UART_RxCpltCallback` 写入环形缓冲区后调用 `osEventFlagsSet(uart1_rx_eventHandle, UART1_RX_FLAG)`
- `ProtocolParser_Task_Run` 缓冲区空时改为 `osEventFlagsWait(..., osFlagsNoClear, 10)`，零延迟响应

---

### P2 — 代码规范清理

**文件**: `App/app_config.h`, `App/app_task.c`, `App/app_includes.h`

**修改**:
- `ParserState_t` 枚举从函数体内移到 `app_config.h`，调试器可见枚举符号名
- `App_UART_Message_t.cmd` 补充注释：透传至 CAN 总线，由接收端根据 `Command_ID_t` 解释执行
- 清理 `app_includes.h` 中的开发过程残留注释

---

## 修改文件汇总

| 文件 | 修改内容 |
|------|----------|
| `App/ring_buffer.h` | `head`/`tail` 改为 `uint8_t`，更新注释 |
| `App/ring_buffer.c` | 去掉取模运算，改为自然溢出回绕 |
| `App/app_globals.h` | 新增 `uart1_tx_mutexHandle`、`uart1_rx_eventHandle`、`UART1_RX_FLAG` 声明 |
| `App/app_config.h` | 新增 `ParserState_t` 枚举，补充 `cmd` 字段注释 |
| `App/app_task.c` | 新增 `uart1_send()` 封装，替换所有裸 `HAL_UART_Transmit`，移除局部 typedef，改用事件等待 |
| `App/app_includes.h` | 清理残留开发注释 |
| `Core/Src/stm32f1xx_it.c` | `HAL_UART_RxCpltCallback` 中新增 `osEventFlagsSet` 通知 |
| `Core/Src/freertos.c` | CubeMX 生成：创建 `uart1_tx_mutex` 和 `uart1_rx_event` |
