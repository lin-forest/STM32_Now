# ROS2 ↔ CAN 舵轮底盘桥接

让 ROS2 通过 CAN 总线直接控制 `3_MCLM_t2` 电机控制器，跳过 `5_ChassisController_t1` 网关层，实现 "上位机（PC）→ CAN → 电机控制器" 的直连架构。

---

## 目录

1. [问题与动机](#一问题与动机)
2. [架构设计](#二架构设计)
3. [CAN 协议速查](#三can-协议速查)
4. [硬件要求](#四硬件要求)
5. [快速开始](#五快速开始)
6. [舵向闭环的完整方案](#六舵向闭环的完整方案)
7. [文件说明](#七文件说明)

---

## 一、问题与动机

### 当前架构的瓶颈

```
ROS2 (PC) ──UART──▶ 5_ChassisController_t1 (STM32网关) ──CAN──▶ 3_MCLM_t2 电机控制器
```

1. **舵向无法闭环**：转向电机目前只有**速度 PID 闭环**（100Hz），缺少**位置/角度闭环**。
   - 舵轮底盘每个动力单元需要知道"当前轮子朝向多少度"
   - MT6701 磁编码器已经调试通过，但**尚未集成到 MCLM_t2 的角度闭环控制中**
   - 参见 `MCLM_t2/doc/Function/Func1_planMt6701.md` 的实施计划

2. **ROS2 集成链路太长**：
   - ROS2 → UART → ChassisController → CAN → MCLM_t2
   - 多层转发增加延迟和调试复杂度

3. **网关层未完成**：`chassis_model.md` 规划的 Phase 2 功能（运动学正解/逆解、CHASSIS_STATE 上报等）尚未实现

### 解决思路

**PC 直连 CAN**：PC 通过 USB-CAN 适配器直接挂到 CAN 总线上，发送 CAN 命令给 MCLM_t2 电机控制器。

```
ROS2 (PC) ──CAN──▶ 3_MCLM_t2 电机控制器（4个动力单元）
                ◀── CAN 状态帧 (20Hz)
```

这样：
- PC 端 Python 负责运动学解算、ROS2 消息转换
- MCLM_t2 保持实时电机控制
- **调试方便**：在 PC 上改算法不用烧录 STM32
- **渐进式**：先跑通速度控制，再扩展角度闭环

---

## 二、架构设计

### 最终架构

```
                        PC (x86_64 / Jetson / 树莓派)
┌──────────────────────────────────────────────────────────────────┐
│  ROS2 Navigation Stack                                           │
│    /cmd_vel (geometry_msgs/Twist)                                │
│         │                                                        │
│  chassis_can_node.py                                             │
│    ├─ 运动学逆解: vx,vy,omega → 各单元驱动速度+转向角度           │
│    ├─ CAN 命令封装 → python-can 发送                             │
│    └─ CAN 状态帧接收 → ROS2 topic 发布                          │
└──────────────────────────┬───────────────────────────────────────┘
                           │ USB (CANable / PCAN / USB2CAN)
                           ▼
                    ┌──────────────┐
                    │  CAN Bus     │  1Mbps
                    │  0x121~0x328 │
                    └──────┬───────┘
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
    MCLM #1 (0x121/2)  MCLM #2 (0x123/4)  MCLM #3 (0x125/6)  MCLM #4 (0x127/8)
    动力单元 1           动力单元 2         动力单元 3           动力单元 4
    (前左 转向+驱动)      (前右 转向+驱动)    (后左 转向+驱动)     (后右 转向+驱动)
```

### 数据流

```
下行（PC → 电机）:
  ROS2 /cmd_vel → 运动学逆解 → set_speed(unit, 'turn', speed)
                             → set_speed(unit, 'power', speed)
                             或（角度模式）
                             → set_steering_angle(unit, angle)
                             → set_speed(unit, 'power', speed)

上行（电机 → PC）:
  MCLM_t2 CAN 状态帧 (0x321~0x328, 20Hz)
    → mclm_can.py 解析为 MotorStatus
    → chassis_can_node.py 发布 /chassis_status
```

### 两种控制模式

| 模式 | 转向电机控制方式 | MCLM 要求 | 适用阶段 |
|------|------------------|-----------|---------|
| **速度模式** | 发速度指令(−100~100)，PC 端推算转向速度 | 现有固件即可 | 当前可用 |
| **角度模式** | 发目标角度(0~360°)，MCLM 内部做位置闭环 | 需实现 Func1_planMt6701.md | 待开发 |

---

## 三、CAN 协议速查

### 命令帧（PC → MCLM）

| 目的 | CAN ID | data[0] | data[1] | data[2] | data[3] | data[4] |
|------|--------|:-------:|:-------:|:-------:|:-------:|:-------:|
| 设置速度 | `0x12X` | `0x11` | speed | - | - | - |
| 停止 | `0x12X` | `0x08` | - | - | - | - |
| 查询状态 | `0x22X` | `0x01` | - | - | - | - |
| 设置角度 | `0x12X` | `0x12` | motor_id | angle_L | angle_H | - |
| 角度模式 | `0x12X` | `0x13` | motor_id | - | - | mode |
| 全车停止 | `0x101` | `0x08` | - | - | - | - |

> `0x12X` 中的 X: 转向命令 ID 末位（1/3/5/7 对应四个单元）
> speed: int8, -100~100
> angle: uint16 LE, x10 精度 (0~3600 对应 0.0°~360.0°)
> mode: 0=速度模式, 1=角度模式

### 状态帧（MCLM → PC, 20Hz 主动上报）

| CAN ID | 含义 |
|--------|------|
| `0x321` | 单元1 转向状态 |
| `0x322` | 单元1 驱动状态 |
| `0x323` | 单元2 转向状态 |
| ... | ... |
| `0x328` | 单元4 驱动状态 |

**标准帧格式（8 字节）**：

```
[0-1] current_logic_speed  (int16 LE, -100~100)  当前实际速度
[2-3] accumulated_ticks    (uint16 LE)             编码器里程计脉冲
[4-5] pwm_output           (int16 LE)             当前 PWM 输出值
[6]   target_logic_speed   (int8)                  目标速度
[7]   flags                (uint8)                 状态标志
  bit0: STALL (堵转)
  bit1: SATURATED (PWM 饱和)
  bit7: ANGLE_MODE (角度模式, 扩展帧标志)
```

**扩展帧（角度模式，仅转向电机）**：

```
[0-1] current_logic_speed  (int16 LE)
[2-3] current_angle        (uint16 LE, 0.1°精度)  当前角度
[4-5] pwm_output           (int16 LE)
[6]   target_angle         (uint8, 1°精度)          目标角度
[7]   flags | (angle_mode << 7)
```

---

## 四、硬件要求

### PC 端 CAN 适配器

| 适配器 | Linux 接口 | 推荐场景 |
|--------|-----------|---------|
| CANable v1/v2 | `slcan0` | 入门/调试，便宜 （~¥100） |
| PCAN-USB | `pcanusb0` | 工业级，稳定 |
| USB2CAN (DIY) | `can0` | 开发板方案 |
| 树莓派 MCP2515 | `can0` | 嵌入式 ROS2 主机 |

### 接线

```
PC ──USB── CANable ──CAN_H(黄色)──▶ CAN Bus
                    ──CAN_L(绿色)──▶ CAN Bus
                    ──GND──────────▶ CAN GND

CAN Bus:
  CAN_H ──┬── MCLM #1 CAN_H ──┬── MCLM #2 CAN_H ──┬── ... ── 120Ω 终端电阻
  CAN_L ──┴── MCLM #1 CAN_L ──┴── MCLM #2 CAN_L ──┴── ... ── 120Ω 终端电阻
```

> ⚠ CAN 总线两端各需一个 120Ω 终端电阻。如果 MCLM 板没有内置，需要在最远的两端外接。

---

## 五、快速开始

### 1. 安装依赖

```bash
# CAN 协议库
pip install python-can

# ROS2 节点（如果没有 ROS2，跳过此步，直接用测试脚本）
pip install rclpy geometry-msgs   # 或在 ROS2 环境中
```

### 2. 配置 CAN 接口

```bash
# CANable (slcan 模式)
sudo slcand -o -s8 -t hw /dev/ttyUSB0 slcan0  # -s8 = 1Mbps
sudo ip link set slcan0 up

# SocketCAN (支持 native 内核 CAN)
sudo ip link set can0 up type can bitrate 1000000

# 验证
cansniffer can0  # 查看 CAN 总线数据
```

### 3. 测试通信

```bash
cd doc/ros2_can_bridge/

# 无 ROS2 的快速测试
python3 test_direct_can.py --channel can0

# 或启动 ROS2 节点
python3 chassis_can_node.py
```

### 4. 用 ROS2 控制

```bash
# 另一个终端
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# 查看状态
ros2 topic echo /chassis_status
```

---

## 六、舵向闭环的完整方案

"无法实现舵向闭环" 的根本原因是 **MCLM_t2 缺少角度位置环**。解决需要 MCLM 侧和 PC 侧配合。

### 短期方案：PC 端有限闭环（速度模式）

1. MCLM_t2 保持现有速度模式，不改固件
2. PC 端 Python 接收 CAN 状态帧中的 `accumulated_ticks`
3. PC 推算转向角度（编码器 ticks → 角度）
4. PC 根据目标角度发速度指令给转向电机

```
PC 角度推算 → 误差计算 → PID → CAN set_speed → 转向电机
← CAN 状态帧(ticks) ←
```

**缺点**：
- 精度受限于编码器 ticks 到角度的换算
- PC 端 20Hz 状态帧更新，控制频率低
- 没有 MT6701 绝对角度反馈，上电位置未知

### 长期方案：MCLM 端 MT6701 角度闭环

按照 [Func1_planMt6701.md](../../../3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/Function/Func1_planMt6701.md) 在 MCLM_t2 上实现：

```mermaid
graph LR
    A[PC CAN 命令<br>0x12 目标角度] --> B[位置 PID 外环<br>20Hz encoder_task]
    B --> C[速度内环<br>100Hz tb6612_task]
    C --> D[TB6612 PWM]
    D --> E[直流电机]
    E --> F[MT6701 编码器]
    F --> B
```

实施步骤（参照 Func1_planMt6701.md 的 Phase 0→3）：

| Phase | 内容 | 预期耗时 |
|-------|------|---------|
| **Phase 0** | MT6701 SPI 通信裸机验证 | 1 天 |
| **Phase 1** | 集成到 encoder_task，CAN 帧上报角度 | 1 天 |
| **Phase 2** | 位置 PID 角度闭环调试 | 2~3 天 |
| **Phase 3** | 综合测试（模式切换/长稳） | 1 天 |

实现后：
- PC 发 `set_steering_angle(unit, 90.0)` → MCLM 内部闭环到 90°
- 无需 PC 端 PID，实时性由 STM32 保证
- CAN 状态帧自动包含 `current_angle` 字段

### 逐步迁移建议

```
Phase 0 ── 当前状态
  PC ──CAN──▶ MCLM_t2 (速度模式, 无角度闭环)

Phase 1 ── 配管调试
  PC ──CAN──▶ MCLM_t2 (速度模式, 读状态帧 ticks 做 PC 端估算)
  │            └── 安装 MT6701, 用测试脚本验证 SPI 通信
  │            └── 参考 Func1_planMt6701.md Phase 0

Phase 2 ── MCLM 角度闭环
  PC ──CAN──▶ MCLM_t2 (角度模式, MCLM 内部位置闭环)
               └── 实现 Func1_planMt6701.md Phase 1~3

Phase 3 ── ROS2 完全集成
  ROS2 Nav Stack ──CAN──▶ MCLM_t2 (角度模式)
  完整运动学解算, 里程计发布, 异常上报
```

---

## 七、文件说明

| 文件 | 说明 |
|------|------|
| `mclm_can.py` | CAN 协议封装库，包含 CAN 帧发送/接收、状态帧解码、运动学逆解 |
| `chassis_can_node.py` | ROS2 节点，订阅 `/cmd_vel`，发布 `/chassis_status` |
| `test_direct_can.py` | 无 ROS2 依赖的 CAN 通信测试脚本 |

### 与现有项目的关系

```
现有架构:
  PC ←UART→ 5_ChassisController_t1 ←CAN→ 3_MCLM_t2

本方案架构（可并行运行）:
  PC ←CAN→ 3_MCLM_t2  (跳过 ChassisController)

两者不冲突：
  - 调试阶段用 PC 直连 CAN 灵活调参
  - 量产部署可选择恢复 ChassisController 网关模式
  - 或保留 PC 直连 CAN 方案，ChassisController 降级为传感器集线器
```

### 参考文档

- [MCLM CAN 协议详述](../../../3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/deepseek_can.md)
- [MT6701 角度闭环实施计划](../../../3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/Function/Func1_planMt6701.md)
- [底盘构型设计](../../chassis_model.md)
- [项目整体架构](../../architecture.md)
