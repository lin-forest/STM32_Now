# 机械臂控制器接入方案 — 3_SteeringArm

> 基于现有底盘架构，通过 CAN 节点接入新 F103 控制器作为机械臂主控制器
> 日期: 2026-06-23

---

## 一、整体系统架构

### 1.1 现有底盘架构（已确认）

```
上层计算机 (PC / 树莓派 / Jetson)
  ┃ UART (115200, 自定义协议帧)
  ▼
┌──────────────────────────────────────────────┐
│  5_ChassisController_t1 (STM32F103)           │
│  角色：数据处理与整合中心 / UART↔CAN 网关     │
│  FreeRTOS 5任务：ProtocolParser / CommandProcess│
│  / UartToCan / CanRxProcess / Heartbeat       │
└────┬─────────────────────────────────────────┘
     ┃ CAN Bus (500kbps, 标准帧11bit)
     ▼
┌────────────────┐   ┌────────────────────────┐
│ 3_MCLM_t2      │   │ 1_SteeringArm (本方案) │
│ 电机控制器节点  │   │ 机械臂控制器节点 ★新增  │
│ (驱动+转向电机) │   │ (J0~J2 + 夹爪)        │
└────────────────┘   └────────────────────────┘
```

### 1.2 新增机械臂控制器在架构中的位置

```
ChassisController_t1 (网关)
  │ 收到上层 UART 命令 → 解析 → 分发
  │
  ├─ CAN 0x1XX → 已有 MCLM_t2 电机控制器（底盘运动）
  │
  ├─ CAN 0x1XX → 1_SteeringArm **本方案**（机械臂控制）
  │     ┌──────────────────────────────────────┐
  │     │ 1_SteeringArm (STM32F103)             │
  │     │ FreeRTOS (CMSIS-V2), CubeMX 初始化     │
  │     │                                       │
  │     │ J0: DC电机+TB6612 (复用3_MCLM代码)    │
  │     │ J1: 舵机 + MT6701(SPI绝对值)          │
  │     │ J2: 舵机 + MT6701(SPI绝对值)          │
  │     │ 夹爪: 双舵机                          │
  │     └──────────────────────────────────────┘
  │
  └─ CAN 0x330 ← 机械臂状态上报 (50ms周期)
```

### 1.3 设计原则（继承底盘网关哲学）

1. **网关不感知机械臂细节** — ChassisController 只转发机械臂控制指令和接收状态，不解码关节级数据
2. **机械臂控制器自治** — 关节PID闭环、舵机PWM生成、编码器读取全部在本地完成
3. **CAN协议统一** — 沿用现有标准帧格式、Little-Endian、命令字节映射

---

## 二、机械臂关节分布与驱动需求

### 2.1 关节定义

```
                          [夹爪双舵机]
                             │
                     ┌───────┘
                     │       第二连杆
                   ╱ J2 (舵机+MT6701)
                 ╱
                │
     第一连杆   │    J2 与 J1 平行但不共面（结构设计）
                │
                   ╲ J1 (舵机+MT6701)
                     ╲
                       └───────┐
                               │  第一连杆
                          J0 (DC电机+TB6612)
                          ↑ 转动轴竖直向上
                     ┌────┴────┐
                     │ 底盘（已有│
                     │ 转动盘） │
                     └─────────┘
```

### 2.2 各关节驱动方式

| 关节 | 驱动方式 | 反馈 | 控制周期 | 备注 |
|------|---------|------|---------|------|
| **J0** (底盘旋转) | 直流电机 + TB6612 | 编码器(TIM) | 10ms PID | 可复用 3_MCLM_t2 的 tb6612_DC_task + PID |
| **J1** (肩关节) | 舵机 (标准50Hz PWM) | MT6701 SPI绝对值 | 20ms 位置闭环 | 同轴布置磁编码器 |
| **J2** (肘关节) | 舵机 (标准50Hz PWM) | MT6701 SPI绝对值 | 20ms 位置闭环 | 同轴布置，与J1共用SPI总线 |
| **夹爪** (工具端) | 双舵机 (两指独立) | 无(开环) | 20ms PWM | 直接固定在第二连杆末端 |

> **关节0（J0）** 的转动轴竖直向上，实际上是"腰关节"（Yaw），与底盘原有转动共用机械接口。
> 舵机选用需要支持 **360° 连续旋转** 或 **有限角度** 取决于结构设计。

---

## 三、硬件资源分析

### 3.1 MCU 选型可行性

**STM32F103C8T6 (LQFP48)** 完全满足需求：

| 资源 | 总量 | 预计使用 | 余量 |
|------|------|---------|------|
| **Flash** | 64 KB | ~38-42 KB | ~22-26 KB ✅ |
| **RAM** | 20 KB | ~8-11 KB | ~9-12 KB ✅ |
| **GPIO** | 37 | ~17-19 | ~18-20 ✅ |
| **Timer** | 4 (TIM1~4) | 全部占用 (详见4.2) | 0 余量 ⚠️ 刚好够用 |
| **SPI** | 2 | 1 (SPI1) | SPI2 备用 ✅ |
| **CAN** | 1 | 1 (CAN1) | — ✅ |

### 3.2 Flash 占用估算

| 模块 | 估算占用 | 说明 |
|------|---------|------|
| HAL + CMSIS + FreeRTOS | ~16 KB | 基础框架 |
| CAN 驱动 + 协议层 | ~4 KB | 继承现有 can_filter/command 模式 |
| J0 直流电机 (TB6612 + PID) | ~4 KB | 直接复用 3_MCLM_t2 代码 |
| MT6701 驱动 ×2 (SPI) | ~2 KB | 复用 6_mt6701_spi 已验证代码 |
| 舵机控制 | ~2 KB | 50Hz PWM 生成 + 位置映射 |
| 夹爪控制 | ~1 KB | 双舵机开环控制 |
| 应用层任务 | ~5 KB | 状态机、CAN 命令处理 |
| printf 浮点支持 | **~12 KB** (可选) | 调试用，正式版可去掉 |
| **合计** | **~34-46 KB** | 不超过 64 KB ✅ |

### 3.3 RAM 占用估算

| 占用项 | 估算 | 说明 |
|-------|------|------|
| FreeRTOS 内核 + HAL | ~3 KB | |
| 任务栈 (6 tasks) | ~4-6 KB | 每任务 128-256 words |
| 应用数据 (关节状态/PID/队列) | ~2 KB | |
| **合计** | **~9-11 KB** | 不超过 20 KB ✅ |

---

## 四、引脚分配方案

### 4.1 完整引脚表

| 引脚 | 功能 | 外设 | 归属 |
|------|------|------|------|
| **PA0** | TIM2_CH1 | **J0 编码器 A** | J0 |
| **PA1** | TIM2_CH2 | **J0 编码器 B** | J0 |
| PA2 | GPIO OUT | J0_IN1 (TB6612方向) | J0 |
| PA3 | GPIO OUT | J0_IN2 (TB6612方向) | J0 |
| **PA5** | SPI1_SCK | **MT6701 共用 SCK** | J1/J2 |
| **PA6** | SPI1_MISO | **MT6701 共用 MISO** | J1/J2 |
| PA7 | SPI1_MOSI | MOSI (未用但CubeMX自动分配) | — |
| **PA9** | USART1_TX | **调试串口** | 调试 |
| **PA10** | USART1_RX | **调试串口** | 调试 |
| **PA11** | CAN1_RX | **CAN 总线** | 通信 |
| **PA12** | CAN1_TX | **CAN 总线** | 通信 |
| **PA8** | TIM1_CH1 | **J0 PWM** (TB6612) | J0 |
| **PB1** | GPIO OUT | **J1_CS** (MT6701片选) | J1 |
| **PB6** | TIM4_CH1 | **J1 舵机 PWM** | J1 |
| **PB7** | TIM4_CH2 | **J2 舵机 PWM** | J2 |
| **PB8** | TIM4_CH3 | **夹爪舵机1 PWM** | 夹爪 |
| **PB9** | TIM4_CH4 | **夹爪舵机2 PWM** | 夹爪 |
| PB12 | GPIO OUT | **J2_CS** (MT6701片选) | J2 |
| **PC13** | GPIO OUT | **板载 LED** (心跳) | 系统 |

> **共 16 个功能性 GPIO** + 1 个 LED = 17 个，LQFP48 可用 37 个，余量充足。
> ⚠️ **定时器刚好用完**：TIM1→J0 PWM, TIM2→编码器, TIM3→HAL时间基准, TIM4→舵机，一个不剩。
> J1_CS 和 J2_CS 是普通 GPIO 软件控制。

### 4.2 定时器资源分配 ⚠️ 关键修正

> **F103C8T6 只有 4 个定时器（TIM1~4），不存在 TIM6/TIM7。**
> 启用 FreeRTOS 后 SysTick 被 RTOS 占用，HAL 需要一个独立 TIM 做时间基准（`HAL_InitTick`）。
> **4 个定时器刚好用完，一个不剩。**

| TIM | 模式 | GPIO | 用途 | 为什么 |
|-----|------|------|------|-------|
| **TIM3** | 基本定时(无引脚) | 无 | **HAL 时间基准** | 纯内部定时，不占用任何引脚 |
| **TIM2** | Encoder mode | PA0, PA1 | **J0 编码器** | 2 通道输入捕捉，标准用法 |
| **TIM1_CH1** | PWM ~20kHz | **PA8** | **J0 直流电机** | APB2=72MHz 精度高，PA8 无冲突 |
| **TIM4_CH1~4** | PWM 50Hz | **PB6~PB9** | **4路舵机** | 唯一 4 通道全空闲的定时器 |

**引脚冲突检查（为什么不能换）：**
- 舵机 PWM 必须用 TIM4：TIM1_CH4(PA11)=CAN1_RX ✗, TIM1_CH2(PA9)=USART1_TX ✗, TIM1_CH3(PA10)=USART1_RX ✗ → 只有 TIM4 能提供 4 个无冲突 PWM 通道
- J0 PWM 用 TIM1_CH1(PA8)：PA8 空闲，无任何外设冲突 ✅
- J0 编码器用 TIM2(PA0,PA1)：标准用法，无冲突 ✅
- HAL 时间基准用 TIM3：纯内部定时，释放其他定时器给应用 ✅

> TIM4 4 通道分别驱动 J1舵机、J2舵机、夹爪1、夹爪2，共用同一时基（50Hz）。

### 4.3 SPI 总线共享方案

两个 MT6701 共用 SPI1 总线，通过独立 CS 分时访问：

```
SPI1 总线 (PA5 SCK, PA6 MISO, PA7 MOSI)
  ├─ J1_CS (PB1) → J1 MT6701
  └─ J2_CS (PB12) → J2 MT6701
```

读取时序：
```c
// 读 J1 角度
HAL_GPIO_WritePin(J1_CS_GPIO_Port, J1_CS_Pin, GPIO_PIN_RESET);
HAL_SPI_Receive(&hspi1, rx, 2, 100);  // 内部送0xFF
HAL_GPIO_WritePin(J1_CS_GPIO_Port, J1_CS_Pin, GPIO_PIN_SET);
// → 解析 J1 角度

// 读 J2 角度（CS不同，其余相同）
HAL_GPIO_WritePin(J2_CS_GPIO_Port, J2_CS_Pin, GPIO_PIN_RESET);
HAL_SPI_Receive(&hspi1, rx, 2, 100);
HAL_GPIO_WritePin(J2_CS_GPIO_Port, J2_CS_Pin, GPIO_PIN_SET);
// → 解析 J2 角度
```

> SPI 配置复用已验证的 `6_mt6701_spi` 配置：Mode 1, Prescaler 64, 8bit, MSB First
> 详见 [`6_mt6701_spi/doc/build_config_260602.md`](../6_MT6701/6_mt6701_spi/doc/build_config_260602.md)

---

## 五、CubeMX 配置清单（你需要做的）

> 按此清单在 CubeMX 中逐项配置，生成代码后我再添加应用层代码。

### 5.1 RCC (时钟)

| 参数 | 值 |
|------|-----|
| HSE | Crystal/Ceramic Resonator (8MHz) |
| PLL | x9 → 72MHz SYSCLK |
| APB1 | 36MHz (/2) |
| APB2 | 72MHz (/1) |

### 5.2 CAN1

| 参数 | 值 | 说明 |
|------|-----|------|
| Mode | Activate | 引脚 PA11(RX), PA12(TX) |
| Prescaler | 4 | 与现有底盘总线一致 |
| BS1 | 13 TQ | |
| BS2 | 4 TQ | |
| SJW | 1 TQ | |
| Auto Retransmission | Enable | |
| Auto Bus Off | Enable | |
| **NVIC** | 使能 CAN1 RX0 中断 | 优先级 5 |

### 5.3 USART1 (调试串口)

| 参数 | 值 |
|------|-----|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| 引脚 | PA9(TX), PA10(RX) |

### 5.4 SPI1 (MT6701 共用)

| 参数 | 值 | 说明 |
|------|-----|------|
| Mode | Full Duplex Master | |
| Data Size | 8 Bits | |
| First Bit | MSB First | |
| Prescaler | 64 | 1.125MHz |
| **CPOL** | **Low** | ❗实测必须 Mode 1 |
| **CPHA** | **2 Edge** | ❗实测必须 Mode 1 |
| NSS | Software | |

### 5.5 ⚠️ 关键：HAL 时间基准设置

CubeMX 默认用 **TIM1** 做 HAL 时间基准，但我们需要 TIM1 做 PWM。必须手动改：

```
Project Manager → Project Settings → Pinout → HAL Settings → Timebase Source
```
从 `TIM1` **改为 `TIM3`**

> **理由**：TIM3 做时间基准 = 纯内部定时器中断（1kHz），不占用任何外部引脚。
> 不改的话 TIM1 被 HAL 占用，J0 将没有 PWM 可用。

### 5.6 TIM2 (J0 编码器)

| 参数 | 值 |
|------|-----|
| Mode | Encoder Mode TI1 and TI2 |
| Channel 1 | PA0 — TI1 |
| Channel 2 | PA1 — TI2 |
| Prescaler | 0 |
| Counter Period | 65535 |
| Auto-reload | Enable |
| Slave Mode | Encoder Mode 3 |

### 5.7 TIM1 (J0 直流电机 PWM) — 与 MCLM_t2 一致

| 参数 | 值 | 说明 |
|------|-----|------|
| Channel 1 | PWM Generation CH1 | **PA8** |
| Prescaler | **0** | APB2=72MHz 不分频 |
| Counter Period (ARR) | **7200-1** | 72MHz/7200 = **10kHz** ✅ 与 MCLM_t2 一致 |
| Pulse for CH1 | 0 (初始) | 启动时电机停止 |
| CH1 Polarity | High |

> 使用 MCLM_t2 同款参数 (PSC=0, ARR=7200) 以便直接复用 `PWM_MAX=7200` 的速度映射代码。
> 10kHz 分辨率 7200 级，比 20kHz 的 3600 级精细一倍，对低速平滑控制更有利。

### 5.8 TIM4 (4路舵机 PWM) — 35kg 300° 数字舵机

舵机参数：**500μs~2500μs 对应 0°~300°**

| 参数 | 值 | 说明 |
|------|-----|------|
| Channel 1 | PWM Generation CH1 | PB6 → J1舵机 |
| Channel 2 | PWM Generation CH2 | PB7 → J2舵机 |
| Channel 3 | PWM Generation CH3 | PB8 → 夹爪1 |
| Channel 4 | PWM Generation CH4 | PB9 → 夹爪2 |
| Prescaler | **18-1** | 36MHz/18 = 2MHz → 每tick=0.5μs |
| Counter Period (ARR) | **40000-1** | 2MHz/40000 = **50Hz** ✅ |
| Pulse | 各通道初始 **3000** (150° 中间) |
| Polarity | High |

> **脉宽映射**：CCR=1000→0°(500μs), CCR=3000→150°(1500μs), CCR=5000→300°(2500μs)
> 分辨率 **0.075°/count**（旧方案 PSC=720→3°/count，精度提升40倍）

### 5.9 GPIO

| 引脚 | Mode | Label | 初始值 |
|------|------|-------|-------|
| PA2 | Output Push-Pull | J0_IN1 | Low ← **首次尝试 Low/Low(coast)** |
| PA3 | Output Push-Pull | J0_IN2 | Low ← 见下方说明 |
| PB1 | Output Push-Pull | J1_CS | High |
| PB12 | Output Push-Pull | J2_CS | High |
| PC13 | Output Push-Pull | LED | Low (on) |

> **J0_IN1/J0_IN2 初始电平说明（v1.0 — 首次尝试 Low/Low）：**
>
> TB6612 方向引脚上电到 FreeRTOS 接管之间的瞬态行为：
> - **Low/Low** → Coast（高阻，电机轴可自由转动）
> - **High/High** → Short Brake（制动，电机轴锁死）
>
> 选择 Low/Low 的原因：① J0 是腰关节（绕 Z 轴），重力不产生转矩，无掉落风险 ② 调试阶段需频繁手动掰 J0 找机械零位 ③ 与 3_MCLM_t2 的 TB6612 初始化一致。
> 如果后续实测发现 Coast 导致外力偏转，可改为 High/High。

### 5.10 FreeRTOS (CMSIS-V2)

| 参数 | 值 |
|------|-----|
| Interface | CMSIS_V2 |
| Heap | 16384 (16KB) |
| USE_NEWLIB_REENTRANT | Enable |

---

## 六、FreeRTOS 任务设计

### 6.1 任务总览

| 任务 | 函数名 | 优先级 | 栈(words) | 周期 | 功能 |
|------|--------|-------|-----------|------|------|
| `CAN_Rx_Ta` | `CAN_Rx_Task_Run` | Normal | 256 | 事件驱动 | 解析 CAN RX → JointCmdQueue |
| `DC_Motor_Ta` | `DC_Motor_Task_Run` | Normal1 | 256 | **10ms** | J0 PID闭环（复用 MCLM 代码） |
| `Servo_Ta` | `Servo_Task_Run` | Normal | 256 | **20ms** | J1/J2/夹爪 PWM 更新 + MT6701 读取 |
| `Arm_State_Ta` | `Arm_State_Task_Run` | Normal | 256 | **50ms** | 收集关节状态 → CAN 上报 |
| `Heartbeat_Ta` | `Heartbeat_Task_Run` | Low | 64 | 300ms | LED 翻转 |

### 6.2 任务间通信

```
                      ┌──────────────────────┐
                      │   JointCmdQueue      │
                      │  (Queue × 8)         │
                      │  {joint_id, target,  │
                      │   cmd_type}          │
                      └──────────┬───────────┘
                                 │
          ┌──────────────────────┼──────────────────────┐
          ▼                      ▼                      ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ DC_Motor_Task    │  │ Servo_Task       │  │ Arm_State_Task   │
│ (J0, 10ms)       │  │ (J1/J2/夹爪,20ms) │  │ (CAN上报, 50ms)   │
│ ─ PID计算         │  │ ─ 读取MT6701角度  │  │ ─ 读g_arm_state   │
│ ─ PWM输出        │  │ ─ 舵机位置闭环    │  │ ─ CAN发送状态帧   │
│ ─ 编码器读取      │  │ ─ PWM更新       │  │                  │
└──────┬───────────┘  └──────┬───────────┘  └──────────────────┘
       │                     │
       ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    g_arm_state (全局结构体)                    │
│  ArmState_t { J0.speed, J1.angle, J2.angle, gripper_pos,   │
│               J1.raw_angle, J2.raw_angle, flags }           │
└─────────────────────────────────────────────────────────────┘
       ▲
       │
┌──────────────────┐
│ CAN_Rx_Task      │
│ ─ 解析CAN命令     │
│ ─ 入JointCmdQueue│
│ ─ 更新g_arm_state │
└──────────────────┘
```

### 6.3 CAN_Rx_Task — CAN 接收与命令分发

```
[CAN ISR → CAN RX FIFO0]
  osMessageQueuePut(canRxQueue, msg)  // 消息长度消息
  ▼
CAN_Rx_Task:
  osMessageQueueGet(canRxQueue, &msg, osWaitForever)
  │  switch(msg.id):
  │  ├─ 0x130 (ARM_CMD) → 解析命令字节
  │  │   ├─ joint_id=0 → JointCmdQueue(J0, value)
  │  │   ├─ joint_id=1 → JointCmdQueue(J1, angle)
  │  │   ├─ joint_id=2 → JointCmdQueue(J2, angle)
  │  │   └─ joint_id=3 → JointCmdQueue(Gripper, pos)
  │  ├─ 0x430 (ARM_CONFIG) → 设置参数
  │  └─ 0x230 (ARM_QUERY) → 触发立即状态上报
```

### 6.4 J0 串级控制 (速度内环 + 位置外环)

J0 采用了**双反馈串级控制**架构：

```
                        位置环 (外环, 50ms)        速度环 (内环, 10ms)
                        ┌─────────────┐          ┌─────────────┐
CAN 目标位置 ──────────→ │  位置 PID    │──速度目标─→│  速度 PID    │──PWM──→ 直流电机
                        │  (MT6701反馈) │          │  (霍尔反馈)   │        (自带霍尔)
                        └─────────────┘          └─────────────┘
                              ↑                        ↑
                          从动轮 MT6701              电机轴霍尔 A/B
                              │                        │
                        知道输出轴绝对位置          知道电机即时转速
                        不受齿隙影响快速响应
```

#### 机械结构

```
直流电机 (自带霍尔 A/B)   主动轮      从动轮
  ┌───┐              ⚙️ ────→ ⚙️
  │ M │──────────────→│ 齿轮  │  │      │ ← 同轴安装 MT6701
  └───┘              └──────┘  └──────┘
  ↑                             ↑
霍尔编码器 (TIM2)          MT6701 (SPI 绝对值)
  速度内环反馈                位置外环反馈
```

> 霍尔在**电机轴上**（减速箱前），MT6701 在**从动轮上**（减速箱后）。
> 两者之间存在**齿轮减速比 + 齿隙**。

#### 速度内环 (10ms) — 复用 3_MCLM_t2 代码

直接移植的模块：
- `tb6612_DC_task.c` → PID 控制循环(10ms)
- `pid.c/.h` → PID 控制器
- `motor_DC_tb6612.c/.h` → TB6612 硬件驱动
- `speed_map.c/.h` → ticks↔logic 映射
- `filter.c/.h` → 编码器 IIR 滤波

```c
// 速度环任务 (10ms)
void DC_Motor_Task(void *argument)
{
    // 初始化 TB6612、PID、编码器...
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    for (;;) {
        // 1. 读取霍尔编码器差值
        int32_t ticks = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        float current_speed = ticks_to_logic(ticks);

        // 2. 目标速度来自位置环
        float target_speed = g_arm_state.j0_velocity_target;

        // 3. PID 计算 → PWM 输出
        float output = PID_Update(&speed_pid, target_speed, current_speed);
        TB6612_SetSpeed(&j0_motor, logic_to_pwm(output));

        // 4. 更新状态
        g_arm_state.j0_current_speed = (int16_t)current_speed;

        osDelay(10);  // 10ms
    }
}
```

#### 位置外环 (50ms) — 新增，与舵机同周期

```c
// 位置环，在 Servo_Task 中每 50ms 执行一次
// 放在 servo_task.c 末尾

#define J0_POSITION_KP      2.0f          // 位置 P 增益
#define J0_BACKLASH_DEG     2.0f          // 齿隙角度，需实测标定
#define J0_POSITION_DEADBAND 0.5f         // 位置死区

static void J0_Position_Loop(void)
{
    // 1. 读取从动轮 MT6701 绝对角度
    uint16_t raw = MT6701_ReadRaw(J0_CS_GPIO_Port, J0_CS_Pin);
    float current_pos = raw * 360.0f / 16384.0f;

    // 2. 计算位置误差
    float error = g_arm_state.j0_target_position - current_pos;

    // 3. 齿隙死区处理
    if (fabsf(error) < J0_POSITION_DEADBAND) {
        g_arm_state.j0_velocity_target = 0;   // 死区内停止
        return;
    }

    // 4. P 控制 → 输出目标速度给速度内环
    float target_vel = error * J0_POSITION_KP;

    // 5. 限幅到逻辑速度范围
    if (target_vel > 100.0f) target_vel = 100.0f;
    if (target_vel < -100.0f) target_vel = -100.0f;

    g_arm_state.j0_velocity_target = (int16_t)target_vel;
}
```

> **为什么位置环只用 P？** 积分作用已经在速度内环里了，外环再加 I 容易引入振荡。
> **齿隙死区需要实测标定**：手动来回转动 J0，MT6701 读数不变化的角度范围就是齿隙值。

#### 适配工作

- 电机配置参数从 `app_motor_cfg_*.h` 移植为 J0 专属参数
- CAN 命令 `0x130 joint_id=0` 设置的是**目标位置**，不再直接设速度
- 增加 `g_arm_state.j0_target_position`（位置环目标）和 `g_arm_state.j0_velocity_target`（速度环目标）

### 6.5 Servo_Task — 舵机 + 编码器联合控制 (20ms)

整合两个相关工作：
1. **读 MT6701** — 复用 `6_mt6701_spi` 已验证代码，分时读取 J1/J2 角度
2. **舵机位置闭环** — 将目标角度与实测角度比较，调整 PWM 脉宽
3. **夹爪控制** — 开环，根据目标位置直接输出脉宽

```c
void Servo_Task_Run(void *argument)
{
    Arm_JointCmd_t cmd;
    
    for (;;) {
        // 1. 检查是否有新命令（非阻塞）
        if (osMessageQueueGet(JointCmdQueueHandle, &cmd, NULL, 0) == osOK) {
            if (cmd.joint_id >= 1 && cmd.joint_id <= 3) {
                g_arm_state.target[cmd.joint_id] = cmd.value;
            }
        }
        
        // 2. 读取 J1 MT6701 绝对角度
        MT6701_CS_LOW(J1);
        HAL_SPI_Receive(&hspi1, rx, 2, 100);
        MT6701_CS_HIGH(J1);
        g_arm_state.j1_raw = (rx[0] << 8 | rx[1]) >> 2;
        g_arm_state.j1_deg = g_arm_state.j1_raw * 360.0f / 16384.0f;
        
        // 3. 读取 J2 MT6701 绝对角度
        MT6701_CS_LOW(J2);
        HAL_SPI_Receive(&hspi1, rx, 2, 100);
        MT6701_CS_HIGH(J2);
        g_arm_state.j2_raw = (rx[0] << 8 | rx[1]) >> 2;
        g_arm_state.j2_deg = g_arm_state.j2_raw * 360.0f / 16384.0f;
        
        // 4. 舵机位置修正（简单比例控制或直接位置映射）
        servo_j1_pulse = angle_to_pulse(g_arm_state.target[1], g_arm_state.j1_deg);
        servo_j2_pulse = angle_to_pulse(g_arm_state.target[2], g_arm_state.j2_deg);
        
        // 5. 更新 PWM 脉宽
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, servo_j1_pulse);  // J1
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, servo_j2_pulse);  // J2
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, g_arm_state.gripper_target);  // 夹爪1
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, g_arm_state.gripper_target);  // 夹爪2
        
        osDelay(20);  // 20ms 周期
    }
}
```

### 6.6 Arm_State_Task — 状态上报 (50ms)

参考 `3_MCLM_t2` 的 `command_task.c:send_motor_status()`，通过 CAN 上报机械臂状态：

```
CAN ID: 0x330 (ARM_STATUS)
data[0..1]: J0 current_logic_speed (int16, LE)
data[2..3]: J1 raw_angle  (uint16, 0~16383)
data[4..5]: J2 raw_angle  (uint16, 0~16383)
data[6]:    Gripper position (uint8, 0~100%)
data[7]:    flags (bit0:J0_stall, bit1:J1_valid, bit2:J2_valid)
```

---

## 七、CAN 通信协议设计

### 7.1 CAN ID 规划

在现有底盘 CAN 总线中新增一组 ID：

| 方向 | CAN ID | 长度 | 用途 |
|------|--------|------|------|
| **下发** | **0x130** | 8 | **ARM_CMD** — 关节控制指令 |
| **下发** | **0x430** | 8 | **ARM_CONFIG** — 参数配置 |
| **下发** | **0x230** | 1~8 | **ARM_QUERY** — 状态查询/日志控制 |
| **上报** | **0x330** | 8 | **ARM_STATUS** — 状态上报 (50ms) |

> 0x130/0x430/0x230/0x330 系列，避开现有 0x10X(广播)、0x12X(电机CMD)、0x22X(状态查询)、0x32X(状态反馈) 范围。

### 7.2 ARM_CMD (0x130) 帧格式

```
Byte 0:    命令字节 (0x11=SET_POSITION, 0x08=STOP, 0x01=HOME)
Byte 1:    关节ID (0=J0, 1=J1, 2=J2, 3=Gripper, 0xFF=All)
Byte 2..3: 目标值 (int16, LE)
  - J0: 逻辑速度 -100~100
  - J1/J2: 角度 0~16383 (14bit 编码器值) 或 0~3600 (0.1°)
  - Gripper: 位置 0~1000
Byte 4..7: 保留 (填充0)
```

示例：设置 J1 到 90°
```
0x130 | 11 01 10 0E 00 00 00 00
                   ↑ J1目标 = 0x0E10 = 3600 = 90.0° (0.1°精度)
```

### 7.3 ARM_QUERY (0x230) 查询格式

与 MCLM_t2 的 `0x225/0x226` 查询帧格式一致：

```
Byte 0:    命令字节
  - 0x01 = QUERY_STATUS (请求状态上报)
  - 0x04 = LOG_START    (开始日志)
  - 0x05 = LOG_STOP     (停止日志)
Byte 1..7: 保留 (填充0)
```

> 注意：0x230 不是空帧。**至少 data[0] 携带命令字节**，由机械臂控制器解析并响应。

### 7.4 ARM_STATUS (0x330) 上报格式

```
Byte 0..1: J0 current_logic_speed  (int16 LE)
Byte 2..3: J1 raw_angle            (uint16 LE, 0~16383)
Byte 4..5: J2 raw_angle            (uint16 LE, 0~16383)
Byte 6:    Gripper position        (uint8, 0~200)
Byte 7:    flags
           bit0: J0_STALL
           bit1: J1_IN_RANGE       (舵机在目标角度±容差内)
           bit2: J2_IN_RANGE
           bit3: GRIPPER_CLOSED
           bit4: J0_SATURATED
```

### 7.4 总线兼容性

| 检查项 | 结论 |
|-------|------|
| 波特率 | 500kbps，与现有总线一致 ✅ |
| CAN ID 冲突 | 0x130/0x430/0x230/0x330 未占用 ✅ |
| ChassisController 代码修改 | 需要新增机械臂命令转发（见下文） |

### 7.5 ChassisController 端变更

`5_ChassisController_t1` 需要小幅更新：

1. **增加机械臂命令 UART 协议**：
   - 新增 `CMD_ARM_CONTROL (0x10)` — 机械臂控制命令
   - 新增 `CMD_ARM_GET_STATE (0x11)` — 查询机械臂状态

2. **CommandProcess_Task 增加分支**：
   ```c
   case CMD_ARM_CONTROL:
       // 构建 0x130 CAN 帧，入 canTxQueue
       can_tx.id = 0x130;
       memcpy(can_tx.data, uart_msg.data, 8);
       osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0);
       break;
   ```

3. **CanRxProcess_Task 增加解析**：
   ```c
   if (std_id == 0x330) {
       // 解码机械臂状态 → g_system_state.arm
       decode_arm_status(data, &g_system_state.arm);
   }
   ```

---

## 八、代码复用策略

### 8.1 可直接复用的模块

| 源项目 | 文件 | 复用方式 | 预计改动 |
|-------|------|---------|---------|
| `3_MCLM_t2` | `App/modules/pid.c/.h` | 直接复制 | 无 ❗ |
| `3_MCLM_t2` | `App/modules/filter.c/.h` | 直接复制 | 无 ❗ |
| `3_MCLM_t2` | `App/modules/speed_map.c/.h` | 直接复制 | 无 ❗ |
| `3_MCLM_t2` | `App/drivers/motor_DC_tb6612.c/.h` | 直接复制 | 无 ❗ |
| `3_MCLM_t2` | `App/services/can_filter.c/.h` | 参考+重写ID表 | ID表替换 |
| `3_MCLM_t2` | `App/services/command.h` | 参考 | 队列类型适配 |
| `6_mt6701_spi` | `Core/Src/main.c` MT6701_ReadRaw | 封装成独立 .c/.h | 函数封装 |
| `5_ChassisController` | `App/app_config.h` | 参考 | CAN参数适配 |

### 8.2 需要新写的代码

| 模块 | 估算量 | 复杂度 |
|------|-------|-------|
| `App/tasks/servo_task.c` — 舵机+编码器联合控制 | ~200行 | 中 |
| `App/drivers/servo.c/.h` — 舵机PWM封装 | ~80行 | 低 |
| `App/drivers/mt6701.c/.h` — MT6701驱动封装 | ~80行 | 低(已验证) |
| `App/modules/arm_kinematics.c/.h` — 运动学(可选) | ~100行 | 中 |
| `App/config/app_arm_config.h` — 机械臂配置参数 | ~80行 | 低 |
| `App/tasks/arm_state_task.c` — 状态上报 | ~100行 | 低 |
| `App/tasks/can_rx_task.c` — CAN接收分发 | ~120行 | 低 |

---

## 九、项目目录结构

```
1_SteeringArm/
├── CMakeLists.txt              # 构建配置（参照 MCLM_t2）
├── Core/                       # CubeMX 生成
│   ├── Inc/
│   │   ├── main.h
│   │   ├── FreeRTOSConfig.h
│   │   ├── can.h
│   │   ├── spi.h
│   │   ├── tim.h
│   │   ├── usart.h
│   │   ├── gpio.h
│   │   └── stm32f1xx_it.h
│   └── Src/
│       ├── main.c              # CubeMX 初始化 + 用户代码入口
│       ├── freertos.c          # 队列/互斥锁/任务创建
│       ├── can.c / spi.c / tim.c / usart.c / gpio.c
│       └── stm32f1xx_hal_msp.c
├── App/                        # 应用层
│   ├── config/
│   │   ├── app_config.h        # CAN ID + 全局参数
│   │   ├── app_globals.h       # 全局 extern 声明
│   │   ├── app_includes.h      # 统一头文件入口
│   │   └── app_arm_config.h    # 机械臂参数 (关节范围/速度限制)
│   ├── tasks/
│   │   ├── can_rx_task.c       # CAN 接收分发
│   │   ├── dc_motor_task.c     # J0 直流电机 PID（复用 MCLM）
│   │   ├── servo_task.c        # J1/J2/夹爪舵机控制
│   │   ├── arm_state_task.c    # 50ms CAN 状态上报
│   │   └── heartbeat_task.c    # LED 心跳
│   ├── modules/
│   │   ├── pid.c/.h            # PID 控制器（复用 MCLM）
│   │   ├── filter.c/.h         # IIR 滤波器（复用 MCLM）
│   │   └── speed_map.c/.h      # 速度映射（复用 MCLM）
│   ├── drivers/
│   │   ├── motor_DC_tb6612.c/.h  # TB6612 驱动（复用 MCLM）
│   │   ├── servo.c/.h           # 舵机 PWM 封装
│   │   └── mt6701.c/.h          # MT6701 编码器驱动
│   └── services/
│       ├── can_filter.c/.h      # CAN 滤波/ID 表（参考 MCLM 改写）
│       └── command.h            # 命令/状态结构体
├── Drivers/                    # STM32 HAL
├── Middlewares/                 # FreeRTOS
├── cmake/                      # CMake 工具链
└── doc/                        # 开发文档
```

---

## 十、实施步骤（建议次序）

### Phase 1：基础设施 (CubeMX + FreeRTOS + CAN)

| 步骤 | 内容 | 预估时间 |
|------|------|---------|
| 1.1 | CubeMX 生成基础工程（按第五章配置） | 30 min |
| 1.2 | 验证 FreeRTOS 启动，LED 心跳闪烁 | 15 min |
| 1.3 | 验证 CAN 通信：发送简单 CAN 帧，SavvyCAN 查看 | 30 min |
| 1.4 | 验证 USART1 printf 调试输出 | 15 min |

### Phase 2：J0 直流电机控制

| 步骤 | 内容 | 预估时间 |
|------|------|---------|
| 2.1 | 移植 TB6612 驱动 + PID + 编码器（来自 3_MCLM） | 1h |
| 2.2 | J0 PID 整定，验证编码器反馈 | 1h |
| 2.3 | CAN 控制 J0 转速（来自 CAN 总线命令） | 30 min |

### Phase 3：舵机 + 编码器

| 步骤 | 内容 | 预估时间 |
|------|------|---------|
| 3.1 | 移植 MT6701 SPI 驱动（来自 6_mt6701_spi） | 30 min |
| 3.2 | 验证 J1/J2 编码器角度读取 | 30 min |
| 3.3 | 实现舵机 PWM 控制（TIM4 4通道50Hz） | 30 min |
| 3.4 | 舵机位置闭环（编码器反馈） | 1h |
| 3.5 | 夹爪双舵机开环控制 | 30 min |
| 3.6 | CAN 控制 J1/J2/夹爪位置 | 30 min |

### Phase 4：集成与调试

| 步骤 | 内容 | 预估时间 |
|------|------|---------|
| 4.1 | CAN 总线联调：底盘网关 ↔ 机械臂控制器 | 1h |
| 4.2 | 机械臂动作序列测试 | 1h |
| 4.3 | 异常处理（超时/堵转/通信丢失） | 1h |
| 4.4 | 整车联合调试 | 2h |

---

## 十一、可行性结论

### ✅ 完全可行

| 维度 | 结论 |
|------|------|
| **MCU 资源** | Flash/RAM/GPIO/定时器 均有充足余量 |
| **CAN 总线** | 负载增加有限（每50ms 1帧ARM_STATUS + 按需控制帧），不必担心冲突 |
| **代码复用** | 60%+ 可直接复用现有模块，新代码量可控 |
| **硬件适配** | 标准外设，验证过的驱动，风险低 |
| **时间预估** | 2-3个工作日可完成核心功能，5-7个工作日完成集成调试 |

### ⚠️ 风险点与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| 舵机位置闭环精度受限于 MT6701 更新速率 | 控制精度 | 可接受，MT6701 14bit ≈ 0.022° 分辨率足够 |
| 舵机 PWM + SPI 共用 TIM 中断优先级 | 任务调度延迟 | 确保 SPI 传输期间不阻塞 PWM 输出 |
| ChassisController 需增加机械臂命令转发 | 网关代码修改 | 改动量小，新增 2 个 UART 命令 + 2 个 CAN ID 解析 |
| J0 和 底盘原有旋转电机共享同一机械接口 | 机械冲突 | 确保 J0 和底盘电机不同时驱动同一转轴 |

---

## 附录：硬件接线图

```
STM32F103C8T6 (LQFP48)
┌──────────────────────────────────────┐
│                                      │
│ PA0 ──── J0 编码器 A                 │
│ PA1 ──── J0 编码器 B                 │
│ PA2 ──── J0_IN1 (TB6612方向)         │
│ PA3 ──── J0_IN2 (TB6612方向)         │
│                                      │
│ PA5 ──── SPI1_SCK ─┬─ J1 MT6701     │
│ PA6 ──── SPI1_MISO ─┤  J2 MT6701     │
│ PA7 ──── SPI1_MOSI ─┘  (共用总线)   │
│ PB1 ──── J1_CS (MT6701片选)          │
│ PB12 ─── J2_CS (MT6701片选)          │
│                                      │
│ PA8 ──── J0_PWM (TIM1_CH1 → TB6612)     │
│ PB6 ──── J1舵机 (TIM4_CH1)           │
│ PB7 ──── J2舵机 (TIM4_CH2)           │
│ PB8 ──── 夹爪舵机1 (TIM4_CH3)         │
│ PB9 ──── 夹爪舵机2 (TIM4_CH4)         │
│                                      │
│ PA9 ──── USART1_TX (调试串口)         │
│ PA10 ─── USART1_RX (调试串口)         │
│ PA11 ─── CAN1_RX                     │
│ PA12 ─── CAN1_TX                     │
│                                      │
│ PC13 ─── LED (心跳)                  │
└──────────────────────────────────────┘
```
