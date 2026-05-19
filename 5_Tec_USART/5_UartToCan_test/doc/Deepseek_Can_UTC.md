# CAN 协议文档（桥接器视角）

> 本文档描述 `5_UartToCan_test` 桥接器涉及的 CAN 协议。
> 目标机（电机控制器）的完整协议详见 `3_MCLM_t2/doc/deepseek_can.md`。

---

## 数据流

```
PC (UART)  ←→  5_UartToCan_test (桥接器)  ←→  3_MCLM_t2 (电机控制器)
                     ↕ CAN 总线
```

### 上行：UART → CAN → 目标机

PC 通过 UART 发送命令帧 → 桥接器解析并转发为 CAN 帧 → 目标机电机控制器执行

### 下行：目标机 → CAN → UART

目标机发送 CAN 状态帧 → 桥接器接收并转发为 UART 字符串 → PC 串口终端显示

---

## CAN ID 定义

定义于 `App/app_config.h`：

| 宏 | ID | 方向 | 用途 |
|---|---|---|---|
| `CAN_MOTOR_TURN_CMD_STDID` | **`0x123`** | RX | **转向**电机控制指令 |
| `CAN_MOTOR_POWER_CMD_STDID` | **`0x124`** | RX | **动力**电机控制指令 |
| `CAN_MOTOR_TURN_CMD_STATUS_STDID` | **`0x223`** | RX | **转向**电机状态查询 / 日志控制 |
| `CAN_MOTOR_POWER_CMD_STATUS_STDID` | **`0x224`** | RX | **动力**电机状态查询 / 日志控制 |
| `CAN_MOTOR_TURN_STATUS_STDID` | **`0x323`** | TX | **转向**电机状态反馈 |
| `CAN_MOTOR_POWER_STATUS_STDID` | **`0x324`** | TX | **动力**电机状态反馈 |

---

## 状态帧格式（新格式）

目标机主动上报或响应查询时，发送 8 字节 CAN 数据帧：

| 字节 | 类型 | 说明 |
|---|---|---|
| [0-1] | `int16` LE | `current_logic_speed` — 实际速度 |
| [2-3] | `uint16` LE | `accumulated_ticks` — 里程计低16位 |
| [4-5] | `int16` LE | `pwm_output` — 当前 PWM 值 |
| [6] | `int8` | `target_logic_speed` — 目标速度 (-100~100) |
| [7] | `uint8` | `flags` — 电机状态标志位 |

### flags 位定义

| bit | 宏 | 含义 |
|---|---|---|
| 0 | `MOTOR_FLAG_STALL` | 堵转：有目标速度但电机不转 |
| 1 | `MOTOR_FLAG_SATURATED` | PID 饱和：PWM 已达上限但仍未达到目标 |

---

## 心跳机制

目标机每 50ms 主动发送两帧状态（0x323 + 0x324），约 20Hz。

主控端若超过 100ms 未收到任何状态帧，可判定节点离线。
