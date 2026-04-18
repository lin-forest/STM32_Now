# 电机控制系统分析报告（PID + Encoder）

> 项目路径：`3_MCLM_t2`  
> 分析日期：2026-04-18  
> 驱动选型：TB6612  
> MCU：STM32F103C8T6 / FreeRTOS + cmsis-os v2

---

## 一、系统整体架构

```
UART2 RX 中断
    └─ Logger.c::RxCpltCallback
         └─ Command_ParseString()  → osMessageQueuePut(CommandQueueHandle)
                    │
                    ▼
              Command_Task
              ├─ LOG_START/STOP → g_logger_enabled
              ├─ LIST_STATUS   → Logger_Print (UART)
              ├─ QUERY_STATUS  → CAN Tx
              └─ 电机指令      → MotorQueueHandle
                                      │
                          ┌───────────┴──────────┐
                          ▼                       ▼
                 TB6612_DC_Task(M0)      TB6612_DC_Task(M1)
                 PID_Compute()           PID_Compute()
                 TB6612_Motor_SetSpeed() TB6612_Motor_SetSpeed()
                 → AckQueueHandle        → AckQueueHandle
                                              │
CAN RX 中断                                   ▼
    └─ HAL_CAN_RxFifo0MsgPendingCallback  Ack_Task → Logger_Print → UART2

Encoder_Task (10ms 周期)
    └─ TIM2(M0)/TIM3(M1) 读计数差值
    └─ ticks_to_logic() → current_logic_speed
    └─ osThreadFlagsSet(Logger_TaHandle, 0x01) → Logger_Task 唤醒打印
```

**控制频率**：Encoder_Task `osDelay(10)` → 100 Hz；TB6612_DC_Task `osDelay(10)` → 100 Hz，与编码器任务同频，对齐 `Ts = 0.01f`。

---

## 二、Encoder（编码器）模块分析

### 2.1 实现文件

| 文件 | 作用 |
|------|------|
| `App/tasks/encoder_task.c` | 周期读取 TIM 计数器，计算 diff，写入 `g_motors[i]` |
| `App/modules/filter.h/c` | IIR 低通滤波器 |
| `App/config/app_config.h` | `SPEED_TICKS_MAX=80`、`ENCODER_FILTER_ALPHA=0.1f`、`SPEED_LOGIC_MAX=100` |

### 2.2 核心流程

```c
// encoder_task.c（每 10ms）
int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(encoder_tims[i]);
int16_t diff = (int16_t)(now - last_cnts[i]);   // ① 16位减法自动处理溢出
last_cnts[i] = now;

g_motors[i].current_ticks       = diff;
g_motors[i].current_logic_speed = ticks_to_logic(diff);  // ② 映射到逻辑速度
```

**速度映射关系**（`ticks_to_logic`）：
```
logic_speed = diff * SPEED_LOGIC_MAX / SPEED_TICKS_MAX
            = diff * 100 / 80
            = diff * 1.25
```
即：编码器每个控制周期计到 80 tick → 逻辑速度 100（满速）。

### 2.3 编码器正确性评估

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 16 位溢出处理 | ✅ 正确 | `(int16_t)(now - last_cnts[i])` 利用补码自动处理计数器回绕 |
| 双路独立计数 | ✅ 正确 | `encoder_tims[2]` 数组分别指向 TIM2/TIM3，循环独立处理 |
| Mutex 保护写共享数据 | ✅ 正确 | `osMutexAcquire(motor_mutexHandle)` 包住写操作 |
| 编码器定时器启动 | ✅ 正确 | 任务内 `HAL_TIM_Encoder_Start(TIM_CHANNEL_ALL)` |
| 采样周期对齐 | ✅ 正确 | `osDelay(10)` = 10ms = `PID_TS`，与 PID 采样时间一致 |
| Logger 唤醒 | ✅ 正确 | 仅在 mutex 释放后才 `osThreadFlagsSet`，避免 Logger 读到脏数据 |

### 2.4 ⚠️ 问题：filter.c 单实例静态状态

```c
// filter.c
int16_t iir_filter(int16_t input, float alpha)
{
    static float state = 0.0f;   // ← 全局唯一，两路共用！
    state = alpha * input + (1.0f - alpha) * state;
    return (int16_t)state;
}
```

**问题**：`static float state` 在两路电机交替调用时会相互污染滤波状态。

**当前影响**：`encoder_task.c` 中注释掉了 `// #include "speed_map.h"`，`ticks_to_logic()` 的实现来自别处（可能是内联宏或未找到的文件）。若当前实现不调用 `iir_filter`，则暂无影响；若调用则必须修复。

**修复方案**：
```c
// 改为传入状态指针
int16_t iir_filter_ex(int16_t input, float alpha, float *state)
{
    *state = alpha * input + (1.0f - alpha) * (*state);
    return (int16_t)(*state);
}

// 调用侧：每路电机维护自己的 state
static float enc_state[MOTOR_COUNT] = {0.0f, 0.0f};
g_motors[i].current_logic_speed = iir_filter_ex(raw, ENCODER_FILTER_ALPHA, &enc_state[i]);
```

---

## 三、PID 控制器模块分析

### 3.1 实现文件

| 文件 | 作用 |
|------|------|
| `App/modules/pid.h` | `PID_Controller` 结构体 + API 声明 |
| `App/modules/pid.c` | `PID_Init` / `PID_Compute` / `PID_Reset` / `PID_SetSetpoint` |
| `App/config/app_task.c` | `Motor_PID_Init()` 按电机索引选参数 |
| `App/config/app_config.h` | M1/M2 各自的 Kp/Ki/Kd/Limit/Ts/Alpha 宏 |

### 3.2 PID_Compute 逐步解析

```c
float PID_Compute(PID_Controller *pid, float current_value)
{
    // Step 1: 误差
    error = pid->setpoint - current_value;

    // Step 2: 条件积分 Anti-Windup
    //   预估 = Kp*e + Ki*(integral_old + e*Ts)
    //   若预估已超出 output_limit → 本轮不积分
    float pre_output_estimate = pid->Kp * error
                              + pid->Ki * (pid->integral + error * pid->Ts);
    int output_saturated = (pre_output_estimate >  pid->output_limit)
                        || (pre_output_estimate < -pid->output_limit);
    if (!output_saturated)
        pid->integral += error * pid->Ts;

    // Step 3: 积分限幅
    pid->integral = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);

    // Step 4: 微分项（基于误差差分） + 一阶 IIR 滤波
    float raw_d = (error - pid->prev_error) / pid->Ts;
    pid->last_derivative = (1.0f - pid->derivative_filter_alpha) * pid->last_derivative
                         + pid->derivative_filter_alpha * raw_d;

    // Step 5: 输出 + 限幅
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->last_derivative;
    output = CLAMP(output, -pid->output_limit, pid->output_limit);

    // Step 6: 更新历史
    pid->prev_error = error;
    pid->prev_value = current_value;

    return output;
}
```

### 3.3 PID 算法正确性评估

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 离散化方式 | ✅ 正确 | 位置式 PID，积分用矩形法（前向欧拉）`integral += error * Ts` |
| 微分基于误差 | ✅ 可接受 | `(e[k]-e[k-1])/Ts`，setpoint 突变时有微分冲击，已用滤波缓解 |
| 微分 IIR 滤波 | ✅ 正确 | `last_d = (1-α)*last_d + α*raw_d`，`α=0.3` 适中 |
| Anti-Windup | ✅ 实现 | 预估法：积分更新前先判断输出是否饱和，不饱和才累积 |
| 积分限幅 | ✅ 实现 | 独立于 Anti-Windup，双重保护 |
| 输出限幅 | ✅ 正确 | `CLAMP(output, ±output_limit)` |
| 多实例独立 | ✅ 正确 | 每个 `Motor_t` 含独立 `PID_Controller`，无共享状态 |
| 采样时间 Ts | ✅ 一致 | `PID_TS=0.01f` 与 `osDelay(10)` 对齐 |
| PID_Reset 完整性 | ✅ 正确 | 清零 `integral`、`prev_error`、`last_derivative`、`prev_value` |
| 方向切换处理 | ✅ 正确 | `tb6612_DC_task.c` 中 setpoint 符号反转时调用 `PID_Reset()` |

### 3.4 ⚠️ 问题：Anti-Windup 预估缺少微分项

**当前预估公式**（仅含 P+I）：
```c
float pre_output_estimate = pid->Kp * error
                          + pid->Ki * (pid->integral + error * pid->Ts);
```

**实际输出公式**（含 P+I+D）：
```c
output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->last_derivative;
```

预估时未加 `Kd * last_derivative` 项。当 `Kd` 较大时，预估与实际输出有偏差，Anti-Windup 判断的饱和边界不准确。

**当前影响**：当前 `Kd=0.0025f` 很小，微分项贡献有限，实际影响可接受。

**修复方案**（严格实现）：
```c
float pre_output_estimate = pid->Kp * error
                          + pid->Ki * (pid->integral + error * pid->Ts)
                          + pid->Kd * pid->last_derivative;  // 加入微分预估
```

### 3.5 ⚠️ 问题：当前 PID 参数（M1）存在风险

```c
// app_config.h — Motor 1 当前启用值
#define MOTOR1_PID_KP    0.01362f   // 注释标注"过小，Kp贡献不足"
#define MOTOR1_PID_KI    5.01f      // 注释标注"过大，积分快速饱和"
#define MOTOR1_PID_KD    0.0025f
#define MOTOR1_PID_INTEGRAL_LIMIT  15.0f
#define MOTOR1_PID_OUTPUT_LIMIT   100.0f
```

**问题分析**：

| 问题 | 影响 |
|------|------|
| `Kp=0.01362` 极小 | 比例响应微弱，几乎全靠积分驱动，响应极慢 |
| `Ki=5.01` 偏大 | 积分项 = `Ki * integral`，积分项快速达到 `integral_limit * Ki = 15 * 5.01 = 75.15`，占输出限幅 75%，极易过饱和 |
| Kp/Ki 比例失调 | 正常 PI 控制中，积分时间常数 `Ti = Kp/Ki ≈ 0.0027s`，远小于采样周期（0.01s），积分主导但无比例支撑 |

**建议参数**（参考被注释掉的 Fix 14C 版本）：
```c
#define MOTOR1_PID_KP    0.8f       // 恢复合理比例增益
#define MOTOR1_PID_KI    0.3f       // 降低积分，配合限幅 integral_limit*Ki = 10*0.3=3
#define MOTOR1_PID_KD    0.0025f    // 保持
#define MOTOR1_PID_INTEGRAL_LIMIT  10.0f
```

---

## 四、TB6612_DC_Task 控制流程分析

```
每 10ms：
  1. 非阻塞取 MotorQueue（motor_index 匹配才处理，否则放回+delay(1)）
  2. Mutex 锁内：读 current_logic_speed → 写 target_logic_speed
  3. Mutex 锁外：PID_Compute(current_speed) → 浮点运算，不持锁
  4. TB6612_Motor_SetSpeed(output) → 写 PWM 寄存器
  5. Mutex 锁内：写 pwm_output（用于 Logger）
```

### 4.1 ⚠️ 问题：单队列双任务竞争

两路 `TB6612_DC_Task` 共用同一个 `MotorQueueHandle`，通过 `motor_index` 字段过滤。若收到对方的消息，则放回队列并 `osDelay(1)`：

```c
if (cmdMsg.motor_index != my_idx) {
    osMessageQueuePut(MotorQueueHandle, &cmdMsg, 0, 0);
    osDelay(1);   // ← 轻微的调度浪费，也存在重复取放的风险
}
```

**潜在问题**：队列满时放回失败会丢失指令（`osMessageQueuePut` 超时为 0）。

**改进方案**：改为每路电机独立队列，或在 `CommandMsg_t` 路由时直接分发到正确队列。

### 4.2 PID 计算在 Mutex 外 ✅

PID 浮点计算在 Mutex 外进行，仅在读写共享数据时加锁，符合最小持锁时间原则。

---

## 五、速度映射（ticks_to_logic）

```c
// 正向映射（Encoder → 逻辑速度）
logic_speed = diff * SPEED_LOGIC_MAX / SPEED_TICKS_MAX
// 反向映射（逻辑速度 → PWM，在 TB6612_Motor_SetSpeed 内）
pwmVal = abs(speed) * MaxPWM / MaxSpeed
       = abs(speed) * 999 / 100
```

**数值链**：
```
编码器 diff(tick) → ticks_to_logic → current_logic_speed[0~100]
                                           ↓
                              PID_Compute(setpoint, current) → output[0~100]
                                           ↓
                         TB6612_Motor_SetSpeed(output) → PWM = output*999/100
```
三级映射单位一致，均为 `逻辑速度 [0~100]`，设计清晰。

---

## 六、问题汇总与优先级

| 优先级 | 问题 | 位置 | 影响 | 修复难度 |
|--------|------|------|------|----------|
| 🔴 高 | **M1 PID 参数失调**（Kp 极小/Ki 过大） | `app_config.h` | 控制性能差，积分饱和振荡 | 低（改常量） |
| 🟡 中 | **filter.c 单实例 static state** | `filter.c` | 双路编码器滤波状态相互污染 | 低（加参数） |
| 🟡 中 | **Anti-Windup 预估缺微分项** | `pid.c:48` | Kd 大时饱和判断不准 | 低（加一行） |
| 🟡 中 | **单队列双任务竞争放回丢失** | `tb6612_DC_task.c:31` | 队列满时指令丢失 | 中（重构路由） |
| 🟢 低 | **ticks_to_logic 实现位置不明** | `encoder_task.c` | 若依赖 speed_map.h（已注释）则编译报错 | 低（确认实现） |
| 🟢 低 | **g_logger_enabled 无 Mutex** | `app_task.c` | 严格并发下存在非原子读写 | 低（加保护） |

---

## 七、修复建议代码片段

### 7.1 修复 Anti-Windup 预估（pid.c）

```c
// 修改前（Line 47）
float pre_output_estimate = pid->Kp * error + pid->Ki * (pid->integral + error * pid->Ts);

// 修改后：补充微分项
float pre_output_estimate = pid->Kp * error
                          + pid->Ki * (pid->integral + error * pid->Ts)
                          + pid->Kd * pid->last_derivative;
```

### 7.2 修复 filter.c 多实例问题

```c
// filter.h
int16_t iir_filter_ex(int16_t input, float alpha, float *state);

// filter.c
int16_t iir_filter_ex(int16_t input, float alpha, float *state)
{
    *state = alpha * (float)input + (1.0f - alpha) * (*state);
    return (int16_t)(*state);
}
```

```c
// encoder_task.c 中使用
static float enc_state[MOTOR_COUNT] = {0.0f, 0.0f};
// ...
int16_t raw = (int16_t)(diff * SPEED_LOGIC_MAX / SPEED_TICKS_MAX);
g_motors[i].current_logic_speed = iir_filter_ex(raw, ENCODER_FILTER_ALPHA, &enc_state[i]);
```

### 7.3 推荐 M1 PID 参数（app_config.h）

```c
// 参考 Fix 14C 已注释版本，恢复合理参数
#define MOTOR1_PID_KP                0.8f
#define MOTOR1_PID_KI                0.3f
#define MOTOR1_PID_KD                0.0025f
#define MOTOR1_PID_INTEGRAL_LIMIT    10.0f
#define MOTOR1_PID_OUTPUT_LIMIT      100.0f
#define MOTOR1_PID_TS                0.01f
#define MOTOR1_PID_DERIVATIVE_FILTER_ALPHA 0.3f
```

---

## 八、结论

| 模块 | 整体评价 |
|------|----------|
| **Encoder 读取逻辑** | ✅ 正确。16位溢出、Mutex、双路独立、定时采样均无问题 |
| **PID 算法实现** | ✅ 基本正确。Anti-Windup + 积分限幅 + 微分滤波均已实现，存在一处预估不完整的小缺陷 |
| **PID 参数（M1）** | ❌ 当前参数明显失调，Kp 过小、Ki 过大，需按建议调整 |
| **任务调度设计** | ✅ 基本合理。锁外做浮点计算、锁内最小化操作，结构清晰 |
| **速度映射链** | ✅ 单位一致，三级映射逻辑正确 |

