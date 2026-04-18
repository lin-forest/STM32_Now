# 第六阶段：编码器计数异常 (0~1 跳变) 与校准模式建立

## 现象

串口输出（DMA 已正常工作后）：

```
[12:14:55.490] 80650,65531,0,-5,65534,0,-2
[12:14:55.500] 80660,65531,0,-5,65534,0,-2
```

- `M1_HW`（TIM2 寄存器）固定在 65531，`M1_dTick` 始终为 0
- 手动转动电机时，计数偶发在 0~1 之间跳变，无法正常累积

---

## 根本原因分析

### 问题 A：`int16_t` 截断导致 0~1 跳变（主因）

**文件**: `App/tasks/encoder_task.c`

```c
// 旧代码（错误）
static int16_t last_cnts[MOTOR_COUNT] = {0};
int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(encoder_tims[i]);
int16_t diff = (int16_t)(now - last_cnts[i]);
```

`__HAL_TIM_GET_COUNTER()` 返回 `uint32_t`，TIM2/TIM3 的 ARR = 65535（无符号 16 位）。
强转 `int16_t` 后，计数器值超过 32767 时符号位被置 1，变成大负数。
下一帧 `diff = now - last_cnt`，两个带错误符号的值相减，结果趋近于 0 或 ±1。

**本质**：`int16_t` 的合法范围是 -32768~32767，而编码器计数 0~65535 超出其上半段，
被截断为负数后差值计算失真。

### 问题 B：只记录增量，无绝对计数，不便于校准

旧代码只将 `diff`（每帧差分）写入 `current_ticks`，无法在串口观察到连续递增/递减的脉冲总数，
难以用"转一圈看总计数"的方法标定编码器线数。

### 问题 C：`filter.h` 的 `static` 滤波器状态被两个电机共享

`iir_filter()` 内部使用单个 `static float state`，Motor1 和 Motor2 交替调用时状态互相污染，
滤波结果混乱。（本阶段暂停调用滤波器，待后续单独修复。）

---

## 修复方案

### Fix 13: encoder_task.c — uint16_t 接收计数，新增累积绝对计数

**文件**: `App/tasks/encoder_task.c`

```c
// 修复后
static uint16_t last_cnts[MOTOR_COUNT] = {0};   // ← uint16_t，不截断符号位

uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tims[i]);
int16_t  diff = (int16_t)(now - last_cnts[i]);  // 两 uint16 相减转 int16，自动处理溢出
last_cnts[i]  = now;

g_motors[i].current_ticks       = diff;
g_motors[i].encoder_count      += diff;          // 新增：累积绝对计数
g_motors[i].current_logic_speed = (float)diff;   // 校准期间直接输出增量，不映射
```

**原理**：两个 `uint16_t` 相减后截断为 `int16_t`，利用补码环绕自动正确处理
0→65535 的溢出边界，正转（now > last）和反转（now < last，环绕）均能得到正确带符号增量。

同时注释掉 `speed_map.h` 和 `filter.h` 的 include，移除 `ticks_to_logic()` 调用。

---

### Fix 14: app_globals.h — Motor_t 新增 encoder_count 字段

**文件**: `App/config/app_globals.h`，`Motor_t` 结构体

```c
// 修改前
int32_t  current_ticks;         // 编码器原始计数值

// 修改后
int16_t  current_ticks;         // 本周期编码器增量（有符号，正转为正）
int32_t  encoder_count;         // 累积绝对计数（用于校准编码器线数，可复位）
```

`int32_t encoder_count` 可跨越 `uint16_t` 溢出边界连续累加，不受硬件计数器环绕影响，
可长期记录总脉冲数。

---

### Fix 15: logger_task.c — 输出格式切换为校准模式 CSV

**文件**: `App/tasks/logger_task.c`

旧格式（含 PID 控制量，7个字段）：

```
SysMs, M1_CNT(u32), M1_Target, M1_CurTicks, M1_PWM, M2_CNT(u32), M2_Target, M2_CurTicks, M2_PWM
```

新格式（纯编码器校准，7个字段）：

```
SysMs, M1_HW, M1_dTick, M1_Abs, M2_HW, M2_dTick, M2_Abs
```

| 列 | 类型 | 含义 |
|---|---|---|
| `SysMs` | uint32 | FreeRTOS 系统时间戳 (ms) |
| `M1_HW` | uint16 (0~65535) | TIM2 寄存器当前原始值，实时反映计数器位置 |
| `M1_dTick` | int16 | 10ms 内脉冲增量，正转为正、反转为负 |
| `M1_Abs` | int32 | 软件累积绝对计数，不受硬件环绕影响 |
| `M2_*` | 同上 | 电机 2 |

snprintf 格式符：`"%lu,%u,%d,%ld,%u,%d,%ld\r\n"`

另增加 UART ORE（溢出错误）标志自动清除，防止 DMA TX 状态机死锁：

```c
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
}
```

---

## 编码器线数校准方法

程序运行后，手动转动电机 N 整圈，观察串口 `M1_Abs` 列的变化量：

```
CPR（每转脉冲数）= |ΔM1_Abs| / 转圈数
```

> **注意**：若 CubeMX 编码器模式选择 `TIM_ENCODERMODE_TI12`（双边沿 4 倍频），
> 读到的 `ΔAbs` 已是编码器物理线数 × 4，需除以 4 得到本体 CPR。

标定完成后，将 CPR 值填入 `app_config.h` 的 `SPEED_TICKS_MAX`（或新增专用宏），
再恢复 `ticks_to_logic()` 速度映射。

---

## 执行状态

- [x] Fix 13: encoder_task.c — `uint16_t` 接收计数，新增 `encoder_count` 累积，移除速度映射
- [x] Fix 14: app_globals.h — `Motor_t` 新增 `encoder_count` 字段，`current_ticks` 改为 `int16_t`
- [x] Fix 15: logger_task.c — 切换为编码器校准模式 CSV 输出，补充 ORE 标志清除
