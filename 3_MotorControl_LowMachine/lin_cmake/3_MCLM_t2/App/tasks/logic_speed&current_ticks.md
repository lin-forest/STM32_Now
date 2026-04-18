# 速度变量说明：理想速度 vs 实际速度

## 一览

| 字段 | 类型 | 所在结构体 | 含义 | 读/写 |
|---|---|---|---|---|
| `motor->pid.setpoint` | `float` | `Motor_t.pid` | **理想速度**（PID 目标，CAN 指令写入的权威值） | 电机任务写，PID 读 |
| `motor->target_logic_speed` | `int16_t` | `Motor_t` | 理想速度镜像，每 10ms 从 setpoint 同步 | 供查询/上报/日志 |
| `motor->current_ticks` | `int16_t` | `Motor_t` | **实际速度**（编码器每周期增量） | Encoder 任务写 |
| `motor->measured_speed` | `int16_t` | `Motor_t` | 实际换算速度，作为 PID 的 current_value 输入 | Encoder 任务写 |

---

## CAN 指令写入理想速度的完整路径

```
CAN 总线帧 (StdId=0x125/0x135, rxData[0]=0x11, rxData[1]=速度值)
    ↓
can.c  HAL_CAN_RxFifo0MsgPendingCallback()
    cmdMsg.type  = CAN_CMD_SET_SPEED
    cmdMsg.value = (int16_t)rxData[1]     ← 从 CAN 数据字节提取
    ↓  osMessageQueuePut(CommandQueueHandle)
command_task.c  Command_Task()
    is_motor_cmd() == true
    ↓  osMessageQueuePut(MotorQueueHandle)
tb6612_DC_task.c  TB6612_DC_Task()
    motor->pid.setpoint = (float)cmdMsg.value   ← ✅ 写入理想速度
    motor->target_logic_speed = (int16_t)motor->pid.setpoint  ← 镜像同步
```

---

## PID 控制环中的使用

```c
// tb6612_DC_task.c，每 10ms 执行一次
float current_speed = motor->measured_speed;          // 实际速度输入

if (fabsf(motor->pid.setpoint) > FLT_EPSILON)
{
    float output = PID_Compute(&(motor->pid), current_speed);
    // PID 内部: error = pid->setpoint - current_value
    // 即:       error = 理想速度    - 实际速度

    TB6612_Motor_SetSpeed(&(motor->hardware), (int16_t)output);
    motor->pwm_output = motor->hardware.pwm_output;
}
else
{
    TB6612_Motor_Stop(&(motor->hardware));   // setpoint ≈ 0 → 停车
}
```

---

## target_logic_speed 的用途

`motor->target_logic_speed` 是 `pid.setpoint` 的 `int16_t` 镜像，**不直接参与 PID 运算**，仅用于：

1. **CAN 状态上报**（`CMD_QUERY_STATUS`）  
   → `command_task.c` 将其打包进 `0x325`/`0x335` 帧的 `[0..1]` 字节发送给上位机。

2. **UART 日志/调试输出**  
   → `Ack_task.c` 打印当前状态时读取该字段。

---

## 两个"底层"同名字段说明

`TB6612_Motor_t`（硬件驱动结构体）中也存在 `target_logic_speed`，由
`TB6612_Motor_SetSpeed()` 写入，反映底层实际下发的 PWM 对应速度值，
与 `Motor_t.target_logic_speed`（逻辑层）含义略有区别，注意区分。
