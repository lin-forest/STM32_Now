# 代码审查日志：app_config.h / app_globals.h

- **审查日期**：2026-04-18
- **修复日期**：2026-04-18
- **审查文件**：
  - `App/config/app_config.h`
  - `App/config/app_globals.h`
- **关联文件**：`App/config/app_task.c`、`App/tasks/logger_task.c`、`App/tasks/encoder_task.c`、`App/tasks/command_task.c`

---

## 问题汇总

| 级别 | # | 问题简述 | 文件 | 行号 | 状态 |
|------|---|----------|------|------|------|
| 🔴 严重 | 1 | IBT-4 宏定义含多余字符 `&htim3 1`，编译报错 | app_config.h | 135 | ✅ 已修复 |
| 🔴 严重 | 2 | `MOTOR_COUNT` 在两个文件中重复定义 | app_config.h / app_globals.h | 5 / 29 | ✅ 已修复 |
| 🔴 严重 | 3 | `Motor_t.hardware` 硬编码为 `TB6612_Motor_t`，切换驱动时架构失效 | app_globals.h | 11–17 | ✅ 已修复 |
| 🔴 严重 | 4 | `MOTOR2_DEFAULT_STOP_MODE` 在 TB6612 块内重复定义（第 69 行 & 第 107 行） | app_config.h | 69 / 107 | ✅ 已修复 |
| 🟡 中等 | 5 | AT8236 / IBT4 块完全缺少 `MOTOR2_*` 配置，`MOTOR_COUNT=2` 时编译失败 | app_config.h | 110–153 | ✅ 已修复 |
| 🟡 中等 | 6 | `tx_buf[64]` 临界溢出：logger 最大写入约 62 字节，`enc_abs` 接近 INT32_MAX 时截断 `\r\n` | app_globals.h / app_task.c | 50 / — | ✅ 已修复 |
| 🟡 中等 | 7 | `MOTOR1_PID_INTEGRAL_LIMIT` 注释仍引用旧 Ki=0.3，实际 Ki=10.0，注释严重过时 | app_config.h | 88 | ✅ 已修复 |
| 🟡 中等 | 8 | 章节编号混乱：缺少第 2 节，出现两个"第 3 节" | app_config.h | 27 / 39 | ✅ 已修复 |
| 🟢 轻微 | 9 | `current_logic_speed` 字段名与注释语义矛盾（注释说校准阶段不做映射） | app_globals.h | 22 | ✅ 已修复 |
| 🟢 轻微 | 10 | AT8236 / IBT4 块缺少编码器定时器配置（`MOTOR1_ENCODER_TIM` 等） | app_config.h | 110–153 | ✅ 已修复 |

---

## 修复记录（2026-04-18）

---

### 🔴 #1 — IBT-4 宏定义语法错误

**文件**：`app_config.h`

```c
// 修复前
#define MOTOR1_TIM_HANDLE  &htim3 1

// 修复后
#define MOTOR1_TIM_HANDLE  &htim3        // 示例: TIM3
```

---

### 🔴 #2 — `MOTOR_COUNT` 双重定义

**文件**：`app_globals.h`

- 删除 `app_globals.h` 中原有的 `#define MOTOR_COUNT 2`。
- 在 `app_globals.h` 顶部 include 区域改为：

```c
#include "app_config.h"   // 获取 MOTOR_COUNT 及 ACTIVE_MOTOR_DRIVER 等全局配置
```

---

### 🔴 #3 — `Motor_t.hardware` 硬编码驱动类型

**文件**：`app_globals.h`

- 删除原有的 `#include "motor_DC_tb6612.h"` 硬编码引入。
- 引入 `MotorHW_t` 类型别名，随 `ACTIVE_MOTOR_DRIVER` 自动切换：

```c
#if   (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_TB6612)
    #include "motor_DC_tb6612.h"
    typedef TB6612_Motor_t MotorHW_t;
#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_AT8236)
    #include "motor_DC_at8236.h"
    typedef AT8236_Motor_t MotorHW_t;
#elif (ACTIVE_MOTOR_DRIVER == MOTOR_DRIVER_IBT4)
    #include "motor_DC_ibt4.h"
    typedef IBT4_Motor_t   MotorHW_t;
#else
    #error "ACTIVE_MOTOR_DRIVER is not set to a recognized value!"
#endif
```

- `Motor_t.hardware` 字段类型由 `TB6612_Motor_t` 改为 `MotorHW_t`。

---

### 🔴 #4 — `MOTOR2_DEFAULT_STOP_MODE` 重复定义

**文件**：`app_config.h`

- 删除原第 107 行（电机2控制限制区）的重复 `#define MOTOR2_DEFAULT_STOP_MODE TB6612_MOTOR_STOP_BRAKE`。
- 保留第 69 行（电机2 GPIO 配置区）的定义，改为注释说明：

```c
// MOTOR2_DEFAULT_STOP_MODE 已在电机2 GPIO 配置区定义（见上方第69行），此处不再重复
```

---

### 🟡 #5 — AT8236 / IBT4 块缺少 MOTOR2 完整配置

**文件**：`app_config.h`

**AT8236 块**新增：

```c
/* 电机 2 (M2) — MOTOR_COUNT=2 时必须定义 */
#define MOTOR2_TIM_HANDLE           &htim4         // 示例: 请按实际连接修改
#define MOTOR2_PWM_CHANNEL1         TIM_CHANNEL_1
#define MOTOR2_PWM_CHANNEL2         TIM_CHANNEL_2
#define MOTOR2_ENCODER_TIM          &htim3
#define MOTOR2_DEFAULT_STOP_MODE    AT8236_STOP_BRAKE

/* 电机 2 控制限制 */
#define MOTOR2_MAX_PWM_OUTPUT       PWM_MAX
#define MOTOR2_MIN_PWM_OUTPUT       0
#define MOTOR2_MAX_SPEED_LOGIC      SPEED_LOGIC_MAX
#define MOTOR2_DEAD_ZONE            5

/* 电机 2 PID 参数 */
#define MOTOR2_PID_KP                0.4584f
#define MOTOR2_PID_KI                17.66f
#define MOTOR2_PID_KD                0.002976f
#define MOTOR2_PID_INTEGRAL_LIMIT    500.0f
#define MOTOR2_PID_OUTPUT_LIMIT      100.0f
#define MOTOR2_PID_TS                0.01f
#define MOTOR2_PID_DERIVATIVE_FILTER_ALPHA 0.3f
```

**IBT4 块**同上补全（`MOTOR2_DEAD_ZONE = 10`，`STOP_MODE = IBT4_STOP_BRAKE`）。

---

### 🟡 #6 — `tx_buf[64]` 临界截断风险

**文件**：`app_task.c` + `app_globals.h`

```c
// app_task.c（定义）
// 修复前
uint8_t tx_buf[64];

// 修复后
uint8_t tx_buf[128];   // 扩容：logger 最大帧约 62 B，原 64 B 在 enc_abs 极限值时截断 \r\n

// app_globals.h（声明同步）
// 修复前
extern uint8_t tx_buf[64];

// 修复后
extern uint8_t tx_buf[128];
```

---

### 🟡 #7 — 积分限制注释引用过时 Ki 值

**文件**：`app_config.h`

```c
// 修复前
#define MOTOR1_PID_INTEGRAL_LIMIT    10.0f  // integral_limit * Ki = 10*0.3 = 3, 远小于output_limit

// 修复后
#define MOTOR1_PID_INTEGRAL_LIMIT    10.0f  // integral_limit * Ki = 10 * 10.0 = 100，与 OUTPUT_LIMIT 持平
```

---

### 🟡 #8 — 章节编号混乱

**文件**：`app_config.h`

```
修复前结构：
  1. 电机驱动选择
  3. 电机1: 共享控制参数    ← 缺少第2节
  3. 电机1: 驱动器特定配置  ← 重复第3节
  4. 系统与通信配置

修复后结构：
  1. 电机驱动选择
  2. 共享控制参数（SPEED_TICKS_MAX、PWM_MAX 等）
  3. 驱动器特定配置（3.1 TB6612 / 3.2 AT8236 / 3.3 IBT4）
  4. 系统与通信配置
```

---

### 🟢 #9 — `current_logic_speed` 字段语义模糊

**文件**：`app_globals.h`

```c
// 修复前
int16_t current_logic_speed;   // 实际测量速度（校准阶段为原始增量，不做映射）

// 修复后
int16_t measured_speed;        // 测量速度（正常模式为逻辑值，校准模式为原始增量）
```

> ⚠️ **后续动作**：需在 `encoder_task.c`、`motor_control_task.c` 等所有引用 `g_motors[i].current_logic_speed` 的地方同步替换为 `measured_speed`。

---

### 🟢 #10 — AT8236 / IBT4 块缺少编码器配置

**文件**：`app_config.h`

在 AT8236 和 IBT4 的 `#elif` 块中分别补充：

```c
#define MOTOR1_ENCODER_TIM  &htim2   // 编码器定时器（示例）
#define MOTOR2_ENCODER_TIM  &htim3   // 编码器定时器（示例）
```

（随 #5 一并新增，已包含在电机2完整配置中。）

---

## 遗留事项

| 优先级 | 内容 | 状态 |
|--------|------|------|
| 🔴 高 | `measured_speed` 字段改名全库同步（`encoder_task.c`、`command_task.c`、`Ack_task.c`、`at8236_DC_task.c`、`motor_DC_tb6612.h/.c`、`command.h`） | ✅ 已完成（2026-04-18）|
| 🟡 中 | AT8236 / IBT4 引脚核实：`MOTOR2_TIM_HANDLE`（`&htim4`）、`MOTOR2_ENCODER_TIM`（`&htim3`）为占位示例，须按实际电路确认 | ⬜ 待确认 |
| 🟡 中 | `app_task.c` 初始化块：切换至 AT8236 / IBT4 后，`g_motors[1].hardware` 初始化字段名需与新驱动结构体对齐并验证 | ⬜ 待验证 |

---

*审查由 Claude Code 生成，版本 2026-04-18*  
*修复由 Claude Code 完成，版本 2026-04-18*
