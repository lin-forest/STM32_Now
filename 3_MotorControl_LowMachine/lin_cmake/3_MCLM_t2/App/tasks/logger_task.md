# logger_task.c 说明文档

## 职责

`Logger_Task` 是串口日志任务，负责将编码器原始数据与电机测量速度以 CSV 格式通过 UART1 DMA 发送，供上位机（Serial Studio / Python / 串口助手）实时解析。

---

## 运行机制

| 步骤 | 说明 |
|------|------|
| 1 | 调用 `Logger_Init()` 完成初始化 |
| 2 | `osThreadFlagsWait(0x01, ...)` 阻塞，等待 `Encoder_Task` 每 10 ms 发出的通知标志 |
| 3 | 检查 `g_logger_enabled`，若为 false 则跳过本轮 |
| 4 | Mutex 外原子读取 TIM2/TIM3 硬件计数寄存器（`hw_cnt`） |
| 5 | 加 Mutex 读取 `g_motors[i]` 共享数据（`current_ticks`、`encoder_count`、`measured_speed`） |
| 6 | `snprintf` 格式化为 CSV 字符串 |
| 7 | 检查 `huart1.gState == HAL_UART_STATE_READY` 后调用 `HAL_UART_Transmit_DMA` |
| 8 | 清除 UART 溢出标志（`UART_FLAG_ORE`）防止死锁 |

---

## 输出格式（CSV）

```
SysMs, M1_HW, M1_dTick, M1_Abs, M1_Speed, M2_HW, M2_dTick, M2_Abs, M2_Speed
```

| 列名 | 类型 | 含义 |
|------|------|------|
| `SysMs` | `uint32` | FreeRTOS 系统 Tick（ms） |
| `M1_HW` | `uint16` | TIM2 计数寄存器原始值（0~65535） |
| `M1_dTick` | `int16` | M1 本周期增量脉冲数（10 ms 内） |
| `M1_Abs` | `int32` | M1 软件累积绝对脉冲数（可跨溢出） |
| `M1_Speed` | `int16` | M1 测量速度（`measured_speed`） |
| `M2_HW` | `uint16` | TIM3 计数寄存器原始值（0~65535） |
| `M2_dTick` | `int16` | M2 本周期增量脉冲数（10 ms 内） |
| `M2_Abs` | `int32` | M2 软件累积绝对脉冲数（可跨溢出） |
| `M2_Speed` | `int16` | M2 测量速度（`measured_speed`） |

---

## 关键局部变量

```c
uint16_t hw_cnt[MOTOR_COUNT];  // TIM 寄存器当前值（Mutex 外原子读）
int16_t  ticks[MOTOR_COUNT];   // 本周期增量（Mutex 内读）
int32_t  enc_abs[MOTOR_COUNT]; // 累积绝对计数（Mutex 内读）
int16_t  mspeed[MOTOR_COUNT];  // 测量速度（Mutex 内读）
```

---

## 变更记录

### 2026-04-18 — 新增 `measured_speed` 输出

- **变量声明**：新增 `int16_t mspeed[MOTOR_COUNT]`
- **数据读取**：Mutex 临界区内新增 `mspeed[i] = g_motors[i].measured_speed`
- **格式字符串**：CSV 列数由 7 列扩展为 9 列，`M1_Speed` 插在 `M1_Abs` 之后，`M2_Speed` 追加在末尾
- **注释**：列说明注释同步更新
