这是被can指令传递到的电机控制器 (3_MCLM_t2) 改动，检查本机文件 @5_UartToCan_test 是否合理

# 电机参数清单

> 核心配置：`App/config/app_config.h`
> 运行时结构体：`App/config/app_globals.h`

---

## 1. 电机驱动选择

| 宏 | 可选值 | 说明 |
|---|---|---|
| `MOTOR_DRIVER_TB6612` | `1` | TB6612 驱动芯片 |
| `MOTOR_DRIVER_AT8236` | `2` | AT8236 驱动芯片 |
| `MOTOR_DRIVER_IBT4` | `3` | IBT-4 驱动模块 |
| `ACTIVE_MOTOR_DRIVER` | 上三者之一 | **当前使用的驱动型号**（在 `app_config.h:19` 修改） |

---

## 2. 共享控制参数（两电机共用）

| 宏 | 默认值 | 说明 |
|---|---|---|
| `SPEED_TICKS_MAX` | `90` | 单控制周期编码器最大计数值（用于速度归一化） |
| `ENCODER_FILTER_ALPHA` | `0.1f` | 编码器 IIR 滤波系数，越小越平滑 |
| `SPEED_LOGIC_MAX` | `100` | 逻辑速度最大值（`0-100`） |
| `MOTOR_CMD_DEFAULT_SPEED` | `50.0f` | CMD_FORWARD/REVERSE 默认速度（50% 满量程） |
| `PWM_MAX` | `7200` | PWM 最大值（TIM ARR 值，对应 100% 占空比） |

### SPEED_TICKS_MAX 计算公式

```
SPEED_TICKS_MAX = PPR × 4(TI12倍频) × (电机最高RPM / 60) × 控制周期(0.01s)
```

**当前参数**：PPR=11, TI12=4x, 电机最高RPM=8986（输出轴 478 RPM × 减速比 18.8）
```
11 × 4 × (8986 / 60) × 0.01 ≈ 66  (理论值)
```
> 实际配置为 `90`（`app_config.h:29`），提高上限以适配更高转速或避免异常饱和。

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

## 4. 电机2（动力电机）— TB6612

### 硬件引脚

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR2_TIM_HANDLE` | `&htim1` | PWM 定时器句柄（与电机1 共用） |
| `MOTOR2_TIM_CHANNEL` | `TIM_CHANNEL_2` | PWM 输出通道 |
| `MOTOR2_IN1_PORT/PIN` | `GPIOB, GPIO_PIN_12` | 方向控制引脚1 |
| `MOTOR2_IN2_PORT/PIN` | `GPIOB, GPIO_PIN_13` | 方向控制引脚2 |
| `MOTOR2_ENCODER_TIM` | `&htim3` | 编码器定时器 |

### 控制限制

| 宏 | 默认值 | 说明 |
|---|---|---|
| `MOTOR2_MAX_PWM_OUTPUT` | `PWM_MAX` | 最大 PWM 输出值 |
| `MOTOR2_MAX_SPEED_LOGIC` | `SPEED_LOGIC_MAX` | 最大逻辑速度 |
| `MOTOR2_DEAD_ZONE` | `10` | PWM 死区 |

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

`CAN_ID_GROUP`（`app_config.h:160`）：

| ID 组 | 转向电机 | 动力电机 |
|---|---|---|
| **`1` (0x125/0x126)** | 控制 `0x125`, 状态 `0x325` | 控制 `0x126`, 状态 `0x326` |
| `2` (0x123/0x124) | 控制 `0x123`, 状态 `0x323` | 控制 `0x124`, 状态 `0x324` |

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

### `Motor_t`（`app_globals.h:27`）

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
| `ticks_to_logic(ticks)` | `ticks × 100 / 90` | 编码器计数值 → 逻辑速度 |
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
