# CAN 数据流文档 (v2)

> 基于当前代码实际实现梳理，旧版见 `can.md`（已废弃）。

---

## 架构说明

协议解析逻辑**内嵌于 `can.c` 的中断回调**中（switch 判断），未抽离为独立服务层。

| 层 | 文件 | 职责 |
|---|---|---|
| HAL 层 | `Core/Src/can.c` | 收帧、ID 白名单过滤、协议解析、推 CommandQueue |
| 任务层 | `App/tasks/command_task.c` | 命令路由、CAN TX 响应（状态查询 / 日志控制） |
| 反馈层 | `App/tasks/Ack_task.c` | 消费 AckQueue，UART 打印 ACK/NACK 信息 |

---

## CAN ID 定义

定义于 `App/config/app_config.h`：

| 宏定义 | ID | 方向 | 用途 | 目标电机 |
|---|---|---|---|---|
| `CAN_MOTOR1_CMD_STDID` | `0x125` | RX | 控制指令 | 转向电机 (M1, index=0) |
| `CAN_MOTOR1_CMD_STATUS_STDID` | `0x225` | RX / TX | 查询、日志控制、日志控制 ACK | 转向电机 (M1, index=0) |
| `CAN_MOTOR1_STATUS_STDID` | `0x325` | TX | 状态查询响应 | 转向电机 (M1, index=0) |
| `CAN_MOTOR2_CMD_STDID` | `0x135` | RX | 控制指令 | 动力电机 (M2, index=1) |
| `CAN_MOTOR2_CMD_STATUS_STDID` | `0x235` | RX / TX | 查询、日志控制、日志控制 ACK | 动力电机 (M2, index=1) |
| `CAN_MOTOR2_STATUS_STDID` | `0x335` | TX | 状态查询响应 | 动力电机 (M2, index=1) |
| `CAN_CMD_STOP_STDID` | `0x101` | RX | 全车停止（motor_index 来自 Data[7]） | — |
| `CAN_CMD_TURN_STDID` | `0x102` | RX | 预留转向命令（当前未在白名单中） | — |
| `CAN_CMD_POWER_STDID` | `0x103` | RX | 预留动力命令（当前未在白名单中） | — |

---

## CAN 总线硬件配置

定义于 `App/config/app_config.h`，初始化于 `Core/Src/can.c:MX_CAN_Init()`：

- 实例：`CAN1`，引脚 PA11 (RX, INPUT) / PA12 (TX, AF_PP)
- 波特率：500 kbps（Prescaler=4, BS1=13TQ, BS2=4TQ, SJW=1TQ）
- 过滤器：Bank 0，掩码模式 32bit，掩码全 0（**硬件放行所有 ID，由软件白名单过滤**），FIFO0
- 中断：`USB_LP_CAN1_RX0_IRQn`，优先级 5
- 自动重传：`ENABLE`，AutoBusOff：`ENABLE`

---

## RX 数据流（接收路径）

```
CAN 总线
    │  硬件帧 (StdId + DLC + Data[8])
    ▼
[Core/Src/can.c]
HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    │  HAL_CAN_GetRxMessage() → rxHeader, rxData[8]
    │
    │  ① ID 白名单过滤 (软件):
    │      允许: 0x125 / 0x135 / 0x225 / 0x235 / 0x101
    │      其余 ID → 直接返回，不处理
    │
    │  ② motor_index 确定:
    │      StdId == 0x125 or 0x225  → motor_index = 0 (M1)
    │      StdId == 0x135 or 0x235  → motor_index = 1 (M2)
    │      StdId == 0x101           → motor_index = rxData[7]
    │
    │  ③ 命令字解析 switch(rxData[0]):
    │      0x11 → CAN_CMD_SET_SPEED, value = (int16_t)rxData[1]
    │      0x08 (CAN_CMD_STOP) → CAN_CMD_STOP, value = 0
    │      0x01 (仅 0x225/0x235) → CMD_QUERY_STATUS
    │      0x04 (仅 0x225/0x235) → CMD_LOG_START
    │      0x05 (仅 0x225/0x235) → CMD_LOG_STOP
    │      default → return，丢弃
    │
    │  ④ 若 cmdMsg.type != CMD_NONE:
    │      osMessageQueuePut(CommandQueueHandle, &cmdMsg, 0, 0)
    ▼
[FreeRTOS Queue]
CommandQueueHandle  (CommandMsg_t)
    ▼
[App/tasks/command_task.c]  Command_Task()
    │  osMessageQueueGet(CommandQueueHandle, &cmd, NULL, osWaitForever)
    │  安全检查: idx = (cmd.motor_index < MOTOR_COUNT) ? cmd.motor_index : 0
    │  读 g_motors[idx]（motor_mutexHandle 保护，超时 5ms）
    │
    ├─ cmd.type == CMD_LOG_START / CMD_LOG_STOP
    │       → 设置 g_logger_enabled = 1/0
    │       → CAN TX 反馈帧 (见 TX 数据流: 日志控制 ACK)
    │       → osMessageQueuePut(AckQueueHandle, &ack, ...)
    │
    ├─ cmd.type == CMD_QUERY_STATUS
    │       → CAN TX 响应帧 (见 TX 数据流: 状态查询)
    │       → osMessageQueuePut(AckQueueHandle, &ack, ...)
    │
    ├─ cmd.type == CMD_LIST_STATUS
    │       → 仅生成 AckMsg，由 Ack_Task 通过 UART 输出
    │
    └─ is_motor_cmd(cmd.type) == true
        (CMD_FORWARD / CMD_REVERSE / CMD_STOP / CMD_SET_SPEED
         / CAN_CMD_SET_SPEED / CAN_CMD_STOP)
            → osMessageQueuePut(AckQueueHandle, &ack, ...)
            → osMessageQueuePut(MotorQueueHandle, &cmd, ...)
```

---

## 协议解析（嵌入 `can.c` 中断回调）

文件：`Core/Src/can.c`，函数：`HAL_CAN_RxFifo0MsgPendingCallback()`

**motor_index 确定规则（由 StdId 决定，而非 Data[7]）：**

| StdId | motor_index | 说明 |
|---|---|---|
| `0x125` 或 `0x225` | `0` | 转向电机 (M1) |
| `0x135` 或 `0x235` | `1` | 动力电机 (M2) |
| `0x101` | `rxData[7]` | 底盘级命令，由上位机通过 Data[7] 指定 |

**命令字节映射（switch rxData[0]）：**

| rxData[0] | 附加条件 | 输出 type | 输出 value |
|---|---|---|---|
| `0x11` | 任意白名单 ID | `CAN_CMD_SET_SPEED` | `(int16_t)rxData[1]` |
| `0x08` (`CAN_CMD_STOP`) | 任意白名单 ID | `CAN_CMD_STOP` | `0` |
| `0x01` | StdId == `0x225` 或 `0x235` | `CMD_QUERY_STATUS` | `0` |
| `0x04` | StdId == `0x225` 或 `0x235` | `CMD_LOG_START` | `0` |
| `0x05` | StdId == `0x225` 或 `0x235` | `CMD_LOG_STOP` | `0` |
| 其他 | — | 直接 `return`（丢弃，不进队列） | — |

> **注意：** 旧协议 `CAN_CMD_SET_SPEED`（`0x03`）已注释掉，当前仅支持 `0x11` 作为设置速度命令。

---

## TX 数据流（发送路径）

### 1. 状态查询响应（CMD_QUERY_STATUS）

触发：收到 `0x01` 命令字（StdId `0x225`/`0x235`）

```
[command_task.c] Command_Task
    │  读 g_motors[idx]（mutex 保护）
    │
    │  确定发送 ID:
    │      idx == 0 → CAN_MOTOR1_STATUS_STDID (0x325)
    │      idx == 1 → CAN_MOTOR2_STATUS_STDID (0x335)
    │
    │  构造 txData[8]:
    │      [0..1] = target_logic_speed  (int16_t, little-endian, memcpy)
    │      [2..3] = current_logic_speed (int16_t, little-endian, memcpy)
    │      [4..5] = pwm_output          (int16_t, little-endian, memcpy)
    │      [6]    = 0x00 (reserved)
    │      [7]    = idx  (电机索引标识)
    │
    │  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox)
    ▼
CAN 总线  →  上位机 / 主控
```

### 2. 日志控制 ACK（CMD_LOG_START / CMD_LOG_STOP）

触发：收到 `0x04`/`0x05` 命令字（StdId `0x225`/`0x235`）

```
[command_task.c] Command_Task
    │  g_logger_enabled = (cmd.type == CMD_LOG_START) ? 1 : 0
    │
    │  确定反馈 ID（原路返回）:
    │      idx == 0 → CAN_MOTOR1_CMD_STATUS_STDID (0x225)
    │      idx == 1 → CAN_MOTOR2_CMD_STATUS_STDID (0x235)
    │
    │  构造 txData[8]:
    │      [0] = 0xCF  (Command Feedback 标识码)
    │      [1] = cmd.type (CMD_LOG_START=9 or CMD_LOG_STOP=10)
    │      [2] = g_logger_enabled (0 or 1)
    │      [3..6] = 0x00 (reserved)
    │      [7] = idx  (电机索引标识)
    │
    │  HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox)
    ▼
CAN 总线  →  上位机 / 主控
```

---

## ACK 旁路流（UART 调试）

每条命令在 `command_task.c` 处理后均生成 UART ACK：

```
Command_Task
    │  构造 AckMsg_t ack { .type, .value, .ok, .current_logic_speed, .pwm_output }
    │  osMessageQueuePut(AckQueueHandle, &ack, ...)
    ▼
[App/tasks/Ack_task.c]  Ack_Task
    │  osMessageQueueGet(AckQueueHandle, &ack, NULL, osWaitForever)
    │  switch(ack.type) → snprintf(buf, ACK_MSG_BUF_SIZE, "ACK: ...")
    │  Logger_Print(buf)
    ▼
UART 输出（调试用）
```

---

## 关键数据结构

定义于 `App/services/command.h`：

```c
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD = 1,
    CMD_REVERSE = 2,
    CMD_STOP = 3,
    CMD_SET_SPEED = 4,
    CMD_LIST_STATUS = 5,
    CMD_QUERY_STATUS = 6,
    CAN_CMD_SET_SPEED = 7,
    CAN_CMD_STOP = 8,
    CMD_LOG_START = 9,
    CMD_LOG_STOP = 10,
} CommandType_t;

typedef struct {
    CommandType_t type;    // 命令类型
    int16_t value;         // 参数（速度值等，无参数命令忽略）
    uint8_t motor_index;   // 目标电机索引 (0:M1, 1:M2)
} CommandMsg_t;

typedef struct {
    CommandType_t type;
    int16_t value;
    uint8_t ok;                    // 1=成功, 0=失败
    int16_t current_logic_speed;   // 当前逻辑速度（反馈）
    int16_t pwm_output;            // 当前 PWM 输出（反馈）
    uint8_t motor_index;           // 对应电机索引
} AckMsg_t;
```

---

## 全局队列

定义于 `Core/Src/freertos.c`（CMSIS-OS 句柄）：

| 队列 | 类型 | 生产者 | 消费者 |
|---|---|---|---|
| `CommandQueueHandle` | `CommandMsg_t` | `can.c` 中断回调 / UART 命令解析 | `command_task.c` |
| `MotorQueueHandle` | `CommandMsg_t` | `command_task.c` | 电机控制任务（`tb6612_DC_task.c` 等） |
| `AckQueueHandle` | `AckMsg_t` | `command_task.c` | `Ack_task.c` |

---

## 与旧版 can.md 的主要差异

| 项目 | 旧版 (can.md) | 当前 (can_2.md) |
|---|---|---|
| 协议解析位置 | 抽离为 `command.c: Command_ParseCAN()` | 内嵌于 `can.c` 中断回调 |
| motor_index 来源 | 所有帧均取 `Data[7]` | 由 StdId 决定（0x10x 才取 `Data[7]`） |
| TX ACK 路径 | 仅查询有 CAN TX | 查询用 `0x325/0x335`，日志控制用 `0x225/0x235` (0xCF 标识) |
| `CommandMsg_t` | 无 `motor_index` 字段 | 新增 `motor_index` 字段 |
| `AckMsg_t` | 无 `motor_index` 字段 | 新增 `motor_index` 字段 |
| 日志控制反馈 | 无 CAN TX，仅 UART ACK | 有 CAN TX 反馈帧（0xCF 头） |
