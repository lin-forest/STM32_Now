# 电机结构体参考

> 本文档记录目标机（3_MCLM_t2）的运行时电机结构体，供桥接器解码状态帧时参考。

---

## Motor_t 结构体

定义于目标机 `App/config/app_globals.h`：

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

### flags 位定义

| 宏 | 值 | 说明 |
|---|---|---|
| `MOTOR_FLAG_STALL` | `0x01` | 堵转：setpoint != 0 但 speed ≈ 0 持续 > 50ms |
| `MOTOR_FLAG_SATURATED` | `0x02` | 饱和：PWM 已达上限但仍无法达到目标速度 |

---

## 状态帧 → 结构体映射

状态帧中 `[7]` 字节的 `flags` 字段对应 `Motor_t.flags`。

桥接器收到 CAN 状态帧（0x323 / 0x324）时解码为结构体字段。
