---
name: utc-gateway-fixes
description: 5_UTC (UART<->CAN网关) 的 CAN ID 扩展、UART 瓶颈分析、状态帧限频、丢帧检测等系列修复
metadata: 
  node_type: memory
  type: project
  originSessionId: c73f1e7d-5539-4833-912f-5ec663c3d372
---

## 5_UTC 网关系列改动

### 背景
5_UTC 是 UART↔CAN 透明网关，连接上位机(UART)与电机控制器 MCLM (CAN)。MCLM 有两个 ID 组：Group1 (0x125/0x126) 和 Group2 (0x123/0x124)，每组各控制转向+动力两个电机。

### 已完成的改动

#### 1. CAN ID 双组支持
- **文件**: `App/app_config.h`
- 原只定义了 Group2 (0x123/0x124/0x323/0x324)，补全 Group1 (0x125/0x126/0x325/0x326)
- 命名加 `_G1`/`_G2` 后缀区分

#### 2. 状态帧解码扩展
- **文件**: `App/app_task.c` — `CanRxProcess_Task_Run()`
- 解码从 2 个 ID 扩展到 4 个: 0x323/0x324/0x325/0x326
- 标签带组号: `TURN(G2)`, `POWER(G1)` 等

#### 3. P0 — 诊断不阻塞 CAN 发送（关键修复）
- **文件**: `App/app_task.c` — `UartToCan_Task_Run()`
- 原：UART 诊断打印在 `HAL_CAN_AddTxMessage` 之前，持 UART mutex 7ms 阻塞 CAN 发送
- 改为：先 CAN 发送，成功时不打印，仅失败时打印错误
- 消除"上位机命令发不出"问题

#### 4. P1 — 状态帧限频输出
- **文件**: `App/app_task.c` — `CanRxProcess_Task_Run()`
- 原：每帧都打印，UART 带宽被占满（50ms×4帧=7ms/帧=56% 负载）
- 改为：帧计数器每 10 帧输出一次（500ms 心跳），UART 负载降到 ~6%
- 注意：最初尝试值比较+超时的方案因编译器优化问题无效，改用计数器方式

#### 5. P2 — CAN RX 丢帧检测
- **文件**: `stm32f1xx_it.c` + `app_globals.h` + `app_task.c`
- ISR 中 `osMessageQueuePut` 失败时递增 `g_can_rx_drop_count`
- `CanRxProcess_Task_Run` 在每帧处理后检查并报告

### 任务调度架构
| 任务 | 优先级 | 栈 |
|---|---|---|
| `ProtocolParser_` | Normal1 (最高) | 1024B |
| `UartToCan_Ta` | Normal | 2048B |
| `CanRxProcess_Ta` | Normal | 2048B |
| `Heartbeat_Ta` | Low | 256B |

### 已知未解决问题
- UART 主机的数据格式与 `ProtocolParser` 的二进制协议(SOF=0xAA)不匹配，产生 `0x04024012` 等乱码 ID
- CAN TX 失败 (`HAL_CAN_AddTxMessage` 返回非 `HAL_OK`) 时消息直接丢弃，无重试
- 无 CAN bus-off 检测和软件恢复逻辑
