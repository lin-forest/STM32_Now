# 电机振荡现象分析：`cansend 125#1155` 根因报告

> 生成日期：2026-04-18
> 关联文件：`app_config.h`, `encoder_task.c`, `tb6612_DC_task.c`, `pid.c`, `motor_DC_tb6612.c`

---

## 1. 命令解析

```
cansend can0 125#1155
         │       │└─ rxData[1] = 0x55 = 85 (十进制) → PID setpoint
         │       └── rxData[0] = 0x11 → CAN_CMD_SET_SPEED
         └────────── StdId = 0x125 → motor_index = 0 (M1)
```

| 字段 | 值 |
|---|---|
| `pid.setpoint` | **85.0f** |
| `setpoint` 的单位 | **原始 encoder ticks / 10ms** |
| `SPEED_TICKS_MAX`（物理最大值） | 90 |

---

## 2. 全链路数据流（当前实际状态）

```
Encoder_Task (10ms)
  uint16_t now  = TIM_CNT (0~65535)
  int16_t  diff = (int16_t)(now - last)   ← 正确处理16位溢出
  g_motors[i].current_ticks       = diff
  g_motors[i].encoder_count      += diff
  g_motors[i].current_logic_speed = (float)diff   ← 原始ticks，无归一化
         │
         ▼ mutex 保护
TB6612_DC_Task (10ms)
  current_speed = motor->current_logic_speed  ← float，实为 ticks/10ms，范围 0~90
  output = PID_Compute(&pid, current_speed)   ← setpoint=85, measurement=ticks
  TB6612_Motor_SetSpeed(&hardware, (int16_t)output)
         │
         ▼ output 范围 [-100, +100] (output_limit)
TB6612_Motor_SetSpeed
  speed ∈ [-MaxSpeed, +MaxSpeed] = [-100, 100]
  pwmVal = abs(speed) * MaxPWM / MaxSpeed
         = abs(output) * 999 / 100
  if |speed| < DeadZone(10) → speed 拉到 ±DeadZone (不清零，保持方向)
  if speed == 0 → PWM = 0 (电机停止)
```

---

## 3. 振荡根因：死区 > 积分最大贡献

### 关键参数矛盾

| 参数 | 值 | 含义 |
|---|---|---|
| `Ki` | `10.0f` | 积分系数 |
| `integral_limit` | `10.0f` | 积分项上限 |
| **积分最大贡献** | `Ki × integral_limit = 100.0f` | 积分饱和时对 output 的最大贡献 |
| `output_limit` | `100.0f` | PID 输出钳位 |
| `DEAD_ZONE` | `10` | 死区阈值（speed < 10 时行为特殊） |
| `Kp` | `0.4584f` | 比例系数 |

> ⚠️ 注意：`MOTOR1_PID_KI` 已从注释中的 `0.5f` 改为 `10.0f`，
> 但 `integral_limit` 注释仍写着 `10*0.3=3`，**注释与代码不一致**。

### Anti-Windup 实际触发分析

```c
pre_output_estimate = Kp * error + Ki * (integral + error * Ts)
                    = 0.4584 * error + 10.0 * (integral + error * 0.01)
```

setpoint=85，当 error=85（启动时）：
```
pre_output = 0.4584×85 + 10×(0 + 85×0.01)
           = 38.96 + 10×0.85
           = 38.96 + 8.5 = 47.46   < 100
→ 未触发 anti-windup，integral 继续累积
```

当 integral 达到上限 10，error=0（电机到达 setpoint）：
```
output = Kp×0 + Ki×10 + Kd×0
       = 0 + 100 + 0 = 100   → 钳位为 100
```

当 integral=10，error 略微正值（e=2）：
```
output ≈ 0.4584×2 + 100 = 100.9 → 钳位 100
```

→ 在 Ki=10 的情况下，**只要 integral > 0，output 就会被 output_limit 钳位**，
  anti-windup 会大量冻结积分更新，系统实际运行在 P-only 模式。

### 振荡机制（四个阶段）

```
阶段①  冲刺（0 → stable speed）
─────────────────────────────────────────────────────────
setpoint=85, current=0 → error=85
pre_output = 47.46 < 100 → integral 累积
integral 迅速到达上限 10.0
output → 100（output_limit）→ PWM = 999
电机快速加速

阶段②  稳定平台
─────────────────────────────────────────────────────────
电机速度接近某平衡点（受负载、惯量决定），速度表现"稳定"
当 current_speed → setpoint=85：
  pre_output = Kp×0 + Ki×(10 + 0×Ts) = 100 → 饱和
  anti-windup 冻结 integral
  output = 100，PWM = 999 不变 —— 仍在冲刺，速度持续上升

若电机继续超调 current_speed > 85（接近物理极限 90）：
  error < 0
  pre_output = 0.4584×(-5) + 10×(10 + (-5)×0.01) = -2.29 + 99.5 = 97.2
             → 未饱和，integral 可以减小
  integral -= 5×0.01 = 0.05 / 周期，缓慢下降

阶段③  跌零触发（output 降到死区以下）
─────────────────────────────────────────────────────────
随着 integral 缓慢从 10 下降，output 也缓慢下降。
当 integral 降到使 output < DEAD_ZONE (10) 的临界点：

  TB6612_Motor_SetSpeed 中：
    if (speed > 0 && speed < DeadZone)  // speed ∈ (0,10)
        speed = DeadZone;               // 拉到 10，不归零

  实际上 output=(int16_t)output → 若 output 在 0~10 之间：
    speed = 10 → pwmVal = 10*999/100 = 99.9 ≈ 99
  这不会立即停车，但电机速度会进一步下降

  当 output ≤ 0（error 变得足够负）：
    speed = 0 → else 分支 → PWM = 0 → 电机制动停止

阶段④  重启
─────────────────────────────────────────────────────────
电机停止 → current_speed = 0 → error = 85 → 回到阶段①
```

---

## 4. "低速时稳定时间更长"的解释

| 参数 | 高速 (setpoint=85) | 低速 (setpoint=42) |
|---|---|---|
| error 初始值 | 85 | 42 |
| 积分充满至 10 所需步数 | ~12 步 (120ms) | ~24 步 (240ms) |
| 超调后 integral 下降速率 | error×Ts = 较大 | error×Ts = 较小 |
| 从 integral=10 降至触发停车 | **更快** | **更慢** |

低速时，超调量小（error 的绝对值小），integral 下降更慢，
在 output 降到死区之前维持时间更长，故第一次稳定期更长。

---

## 5. 当前代码存在的问题汇总

| # | 问题 | 位置 | 严重度 |
|---|---|---|---|
| 1 | `integral_limit` 注释与代码不符（注释说 Ki=0.3，实际 Ki=10.0） | `app_config.h:88` | ⚠️ 中 |
| 2 | `current_logic_speed` 存的是原始 ticks，非逻辑速度，字段名具有误导性 | `encoder_task.c:45` | ⚠️ 中 |
| 3 | `target_logic_speed` 在 TB6612_Motor_Init 中类型为 `int16_t`，但赋值来源为 `float pid.setpoint`，存在精度截断 | `motor_DC_tb6612.c:121` | ⚠️ 中 |
| 4 | logger 当前不输出 PID setpoint / pid output，诊断时缺失关键变量 | `logger_task.c` | ℹ️ 低 |
| 5 | `TB6612_Motor_SetSpeed` 死区逻辑将 0<speed<10 拉到 10，而非停车，与"死区"语义不符 | `motor_DC_tb6612.c:81` | ℹ️ 低 |

---

## 6. 修复建议

### 6.1 归一化 setpoint 和 measurement 到统一域

**encoder_task.c** 中改为归一化：
```c
// 将 diff（ticks/10ms）归一化为逻辑速度 0~100
g_motors[i].current_logic_speed =
    (float)diff * SPEED_LOGIC_MAX / SPEED_TICKS_MAX;
// 例: diff=85 → logic_speed = 85/90*100 ≈ 94.4
```

CAN 命令中的 setpoint (0~255) 也应映射到逻辑域 (0~100)。

### 6.2 Ki 和 integral_limit 参数重新评估

Ki=10.0 搭配 integral_limit=10.0 会导致积分饱和后 output 被
output_limit(100) 直接钳位，anti-windup 几乎不起作用（参见第3节）。

建议先恢复 `Ki = 0.5f`，`integral_limit = 10.0f` 进行调试，
确认稳态是否能保持后再逐步调大 Ki。

### 6.3 更新 integral_limit 注释

```c
// 修改前（错误）
#define MOTOR1_PID_INTEGRAL_LIMIT  10.0f  // integral_limit * Ki = 10*0.3 = 3

// 修改后
#define MOTOR1_PID_INTEGRAL_LIMIT  10.0f  // 积分项上限；Ki×limit = 10*10 = 100 = output_limit
```
