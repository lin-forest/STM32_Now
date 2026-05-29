# 电机参数清单

> 核心配置：`App/config/app_config.h`
> 运行时结构体：`App/config/app_globals.h`

---

## 1. 电机驱动类型定义

| 宏 | 值 | 说明 |
|---|---|---|
| `MOTOR_DRIVER_TB6612` | `1` | TB6612 驱动芯片 |
| `MOTOR_DRIVER_IBT4` | `3` | IBT-4 驱动模块 |

### 预设选择

| 宏 | 可选值 | 说明 |
|---|---|---|
| `MOTOR_CFG_SET` | `1` (DEFAULT) / `2` (NEW) | **唯一选择开关**（`app_config.h`） |

| 预设 | 电机1（转向） | 电机2（动力） | 适用场景 |
|---|---|---|---|
| `MOTOR_CFG_DEFAULT` (1) | TB6612 | TB6612（旧引脚） | 原有配置兼容 |
| `MOTOR_CFG_NEW` (2) | TB6612 | **IBT-4 (BTS7960)** | 新动力电机 |

每个预设文件自包含 `MOTOR1_DRIVER` / `MOTOR2_DRIVER` 宏，由 `app_config.h` 按 `MOTOR_CFG_SET` 选择加载。

---

## 2. 全局共享参数（两电机共用）

定义于 `app_config.h`，所有预设共享。

| 宏 | 值 | 说明 |
|---|---|---|
| `SPEED_LOGIC_MAX` | `100` | 统一的逻辑速度最大值 |
| `PWM_MAX` | `7200` | PWM 最大值（TIM ARR，100% 占空比） |
| `ENCODER_FILTER_ALPHA` | `0.1f` | 编码器 IIR 滤波系数 |
| `MOTOR_CMD_DEFAULT_SPEED` | `50.0f` | CMD_FORWARD/REVERSE 默认速度（50% 满量程） |

| 宏 | 默认值 | 说明 |
|---|---|---|
| `SPEED_TICKS_MAX` | 预设相关 | 单控制周期编码器最大计数值（用于速度归一化） |
| | DEFAULT=`90`, NEW=`96` | 各预设内独立定义 |

### SPEED_TICKS_MAX 计算公式

```
SPEED_TICKS_MAX = PPR × 4(TI12倍频) × (电机最高RPM / 60) × 控制周期(0.01s)
```

**当前参数**：PPR=11, TI12=4x, 电机最高RPM=8986（输出轴 478 RPM × 减速比 18.8）
```
11 × 4 × (8986 / 60) × 0.01 ≈ 66  (理论值)
```
> 当前配置为 `96`（`app_motor_cfg_new.h:22`），提高上限以适配更高转速或避免异常饱和。

---

## 3. 电机1（转向电机）— TB6612

### 硬件引脚

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR1_TIM_HANDLE` | `&htim1` | PWM 定时器句柄 |
| `MOTOR1_TIM_CHANNEL` | `TIM_CHANNEL_1` | PWM 输出通道 |
| `MOTOR1_IN1_PORT/PIN` | `GPIOB, GPIO_PIN_0` | 方向控制引脚1 |
| `MOTOR1_IN2_PORT/PIN` | `GPIOB, GPIO_PIN_1` | 方向控制引脚2 |
| `MOTOR1_DEFAULT_STOP_MODE` | `TB6612_MOTOR_STOP_BRAKE` | 默认停止模式（刹车/滑行） |
| `MOTOR1_ENCODER_TIM` | `&htim2` | 编码器定时器 |

### 控制限制

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR1_MAX_PWM_OUTPUT` | `PWM_MAX` | 最大 PWM 输出值 |
| `MOTOR1_MIN_PWM_OUTPUT` | `0` | 最小 PWM 输出值 |
| `MOTOR1_MAX_SPEED_LOGIC` | `SPEED_LOGIC_MAX` | 最大逻辑速度 |
| `MOTOR1_DEAD_ZONE` | `10` | PWM 死区（低于此值忽略，防抖动） |

### PID 参数

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR1_PID_KP` | `0.4584f` | 比例系数 |
| `MOTOR1_PID_KI` | `17.66f` | 积分系数 |
| `MOTOR1_PID_KD` | `0.0025f` | 微分系数 |
| `MOTOR1_PID_INTEGRAL_LIMIT` | `5.66f` | 积分限幅（Ki × limit ≈ OUTPUT_LIMIT） |
| `MOTOR1_PID_OUTPUT_LIMIT` | `100.0f` | PID 输出限幅（= SPEED_LOGIC_MAX） |
| `MOTOR1_PID_TS` | `0.01f` | 采样周期（10ms） |
| `MOTOR1_PID_DERIVATIVE_FILTER_ALPHA` | `0.3f` | 微分项低通滤波系数 |

---

## 4. 电机2（动力电机）— IBT-4 (BTS7960)

> 仅 `MOTOR_CFG_NEW` 预设生效（`App/config/app_motor_cfg_new.h`）

### TIM 资源分配

| TIM | 模式 | 通道 | 引脚 | 用途 |
|-----|------|------|------|------|
| **TIM1** | PWM (Period=7200) | CH1 | PA8 | **电机1** TB6612 PWM |
| | | CH2 | PA9 | **电机2** IBT4 RPWM (正转) |
| | | CH3 | PA10 | **电机2** IBT4 LPWM (反转) |
| **TIM2** | Encoder | CH1/CH2 | PA0/PA1 | 电机1 编码器 |
| **TIM3** | Encoder | CH1/CH2 | PA6/PA7 | **电机2 编码器** |

### 硬件引脚

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR2_IBT4_TIM` | `&htim1` | PWM 定时器句柄（与电机1 共用 TIM1） |
| `MOTOR2_IBT4_CH_F` | `TIM_CHANNEL_2` | RPWM 通道（正转），PA9 |
| `MOTOR2_IBT4_CH_R` | `TIM_CHANNEL_3` | LPWM 通道（反转），PA10 |
| `MOTOR2_IBT4_EN_PORT/PIN` | `NULL / 0` | 使能引脚（未使用） |
| `MOTOR2_IBT4_POLARITY` | `0` | 极性（0=默认） |
| `MOTOR2_ENCODER_TIM` | `&htim3` | 编码器定时器，纯编码器模式 |

### 控制限制

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR2_MAX_PWM_OUTPUT` | `PWM_MAX` | 最大 PWM 输出值 |
| `MOTOR2_MAX_SPEED_LOGIC` | `SPEED_LOGIC_MAX` | 最大逻辑速度 |
| `MOTOR2_DEAD_ZONE` | `10` | PWM 死区 |

### IBT4 控制逻辑

```
speed > 0  → CH2(RPWM) = pwm, CH3(LPWM) = 0   ← 正转
speed < 0  → CH2(RPWM) = 0,    CH3(LPWM) = pwm ← 反转
speed = 0  → CH2 = 0, CH3 = 0                   ← 停止
```

### PID 参数（可独立整定，默认与电机1相同）

| 宏 | 默认值 |
|---|---|
| `MOTOR2_PID_KP` | `0.4584f` |
| `MOTOR2_PID_KI` | `17.66f` |
| `MOTOR2_PID_KD` | `0.0025f` |
| `MOTOR2_PID_INTEGRAL_LIMIT` | `5.66f` |
| `MOTOR2_PID_OUTPUT_LIMIT` | `100.0f` |
| `MOTOR2_PID_TS` | `0.01f` |
| `MOTOR2_PID_DERIVATIVE_FILTER_ALPHA` | `0.3f` |

---

## 5. CAN 通信参数

### CAN ID 选择

`CAN_ID_GROUP`（`app_config.h:137`）：

| ID 组 | 转向电机 | 动力电机 |
|---|---|---|
| **`1` (0x125/0x126)** | 控制 `0x125`, 状态查询 `0x225`, 状态反馈 `0x325` | 控制 `0x126`, 状态查询 `0x226`, 状态反馈 `0x326` |
| `2` (0x123/0x124) | 控制 `0x123`, 状态查询 `0x223`, 状态反馈 `0x323` | 控制 `0x124`, 状态查询 `0x224`, 状态反馈 `0x324` |

### 总线参数

| 宏 | 默认值 | 说明 |
|---|---|---|
| `CAN_PRESCALER` | `4` | 分频器 |
| `CAN_MODE` | `CAN_MODE_NORMAL` | 工作模式 |
| `CAN_TIME_SEG1` | `CAN_BS1_13TQ` | 时间段1 |
| `CAN_TIME_SEG2` | `CAN_BS2_4TQ` | 时间段2 |

### 命令定义

| 宏 | 值 | 说明 |
|---|---|---|
| `CAN_CMD_SET_SPEED_T2` | `0x11` | 设置速度命令 |
| `CAN_CMD_QUERY_STATUS` | `0x01` | 查询状态 |
| `CAN_CMD_LOG_START` | `0x04` | 开始日志 |
| `CAN_CMD_LOG_STOP` | `0x05` | 停止日志 |
| `CAN_CMD_STOP_STDID` | `0x101` | 全车停止 ID |
| `CAN_CMD_TURN_STDID` | `0x102` | 全车转向 ID |
| `CAN_CMD_POWER_STDID` | `0x103` | 全车动力 ID |

---

## 6. 运行时结构体

### `Motor_t`（`app_globals.h:20`）

```c
typedef struct {
    TB6612_Motor_t hardware;       // 硬件驱动句柄
    PID_Controller pid;            // PID 控制器实例
    float  target_logic_speed;     // 目标逻辑速度 (-100 ~ 100)
    float  current_logic_speed;    // 实际测量速度
    int32_t current_ticks;         // 编码器原始计数值（每周期差值）
    int32_t accumulated_ticks;     // 编码器累计计数值（绝对位置）
    int16_t pwm_output;            // 当前输出的 PWM 值
    uint8_t flags;                 // 电机状态标志 (MOTOR_FLAG_*)
    uint8_t stall_counter;         // 连续堵转周期计数
} Motor_t;

extern Motor_t g_motors[MOTOR_COUNT];  // [0]=转向电机, [1]=动力电机
```

### Motor_t.flags 位定义

| 宏 | 值 | 说明 |
|---|---|---|
| `MOTOR_FLAG_STALL` | `0x01` | 堵转：setpoint != 0 但 speed ≈ 0 持续 > 50ms |
| `MOTOR_FLAG_SATURATED` | `0x02` | 饱和：PWM 已达上限但仍无法达到目标速度 |

检测逻辑在 `tb6612_DC_task.c` 的 PID 控制循环中实现。停止时自动清零。  
通过 CAN 状态帧 `[7]` 字节上报给主控。

> 经 CAN 总线录制验证：真实堵转时正确上报 `0x03`（STALL\|SATURATED），正常运行时 `flags=0x00`。详见 [`ai_session/can_data_analyze.md`](ai_session/can_data_analyze.md)。

### `PID_Controller`（`pid.h:7`）

| 字段 | 说明 |
|---|---|
| `Kp, Ki, Kd` | PID 系数 |
| `setpoint` | 目标值 |
| `integral` | 积分累计值 |
| `integral_limit` | 积分限幅 |
| `output_limit` | 输出限幅 |
| `Ts` | 采样周期 |
| `derivative_filter_alpha` | 微分滤波系数 |

---

## 7. 速度映射（`speed_map.c`）

| 函数 | 公式 | 说明 |
|---|---|---|
| `ticks_to_logic(ticks)` | `ticks × 100 / 96` | 编码器计数值 → 逻辑速度 |
| `logic_to_pwm(logic)` | `logic × 7200 / 100` | 逻辑速度 → PWM 值 |

---

## 快速定位

```bash
# 所有宏定义
grep -rn "#define.*MOTOR\|#define.*PID\|#define.*SPEED\|#define.*PWM" App/config/app_config.h

# PID 参数使用
grep -rn "PID_KP\|PID_KI\|PID_KD\|PID_Init" App/ --include="*.c" --include="*.h"

# 运行时电机对象
grep -rn "g_motors\[" App/ --include="*.c"

# CAN 命令处理
grep -rn "cmdMsg\|CMD_SET_SPEED\|CAN_CMD" App/tasks/ --include="*.c"
```
