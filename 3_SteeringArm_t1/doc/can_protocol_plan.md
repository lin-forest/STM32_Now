# CAN 协议规划 — 3_SteeringArm_t1 多舵机同时控制

> 文档版本: v1.0
> 创建日期: 2026-06-25
> 状态: 规划阶段 ⏳

---

## 1. 背景

当前 `CAN_Rx_Task` 中 0x130 协议一次只能控制一个关节（`data[1]=joint_id`，`data[2:3]=value`）。
要同时控制 J1+J2 必须发两帧 CAN，存在时间差和不同步问题。本方案扩展 CAN 协议，使一帧可控制多关
节同步运动，并完善状态上报、速度独立设置、软件滤波等功能。

参考 `3_MCLM_t2` 的 CAN 架构（ID 分组、命令字节、白名单表 + `app_config.h` 集中配置），结合本项
目多轴协调需求设计。

---

## 2. CAN ID 分配总表

| CAN ID | 方向 | 宏名称 | 用途 | 实现状态 |
|:------:|:----:|:-------|------|:--------:|
| **0x130** | RX (Chassis→Arm) | `CAN_ARM_CMD_STDID` | 关节角度控制（单关节 + 多关节 bitmask） | 扩展中 |
| **0x131** | RX (Chassis→Arm) | `CAN_ARM_VEL_STDID` | 每关节独立速度设置 | 新增 |
| **0x230** | RX (Chassis→Arm) | `CAN_ARM_QUERY_STDID` | 状态查询（触发立即上报） | 新增 |
| **0x330** | TX (Arm→Chassis) | `CAN_ARM_STATUS_STDID` | 周期状态上报（50ms / 20Hz） | 新增 |
| **0x430** | RX (Chassis→Arm) | `CAN_ARM_CONFIG_STDID` | 系统配置（回中/速度/加锁/限位） | 扩展中 |

### 总线负载估算（500kbps）

| 帧 | 周期 | 帧率 | 每帧总线时间 | 占用带宽 |
|:---|:----:|:----:|:-----------:|:--------:|
| 0x330 状态上报 | 50ms | 20 fps | ~0.26ms | ~0.52% |
| 0x130/131/230/430 命令 | 事件触发 | 偶发 | ~0.26ms | 可忽略 |
| **合计** | | | | **< 1%** |

---

## 3. 0x130 — 关节角度控制（核心设计）

用 `data[0]` 作为协议版本鉴别器。旧代码忽略 `data[0]`，故向后兼容。

### 3.1 单关节模式（LEGACY，data[0]==0x11）

```
Byte 0: 0x11          (cmd byte: SET)
Byte 1: joint_id      (1=J1, 2=J2, 3=Gripper)
Byte 2-3: value       (int16 LE, J1/J2=0.1°, Gripper=CCR pulse)
Byte 4-7: 保留 (填0)
```

示例 — J1 转 90°：
```
0x130: 11 01 84 03 00 00 00 00
       │  │  ↑____↑
      SET J1  900 = 90.0°
```

### 3.2 多关节模式（MULTI，data[0]==0x12）

```
Byte 0: 0x12                    (cmd byte: MULTI)
Byte 1: bitmask                 (bit0=J0预留, bit1=J1, bit2=J2, bit3=Gripper)
Byte 2-3: J1 角度 (int16 LE, 0.1°)   [bit1 置位时有效]
Byte 4-5: J2 角度 (int16 LE, 0.1°)   [bit2 置位时有效]
Byte 6:   夹爪位置 (uint8, 0~200)     [bit3 置位时有效]
Byte 7:   J0 速度 (int8, -100~100)    [bit0 置位时有效]
```

bitmask 位域定义：
```c
#define ARM_MASK_J0      0x01
#define ARM_MASK_J1      0x02
#define ARM_MASK_J2      0x04
#define ARM_MASK_GRIPPER 0x08
```

示例 1 — 一帧同时控制 J1→90° + J2→30°：
```
0x130: 12 06 84 03 2C 01 00 00
       │  │  │____│ │____│
     MULTI J1+J2 J1=90° J2=30°
           0x06 = bit1|bit2
```

示例 2 — 只控制夹爪 50% 开度：
```
0x130: 12 08 00 00 00 00 64 00
              │          │
           J1=无效      Gripper=100 (0~200, 50%)
```

### 3.3 解析伪代码

```c
if (msg.id == CAN_ARM_CMD_STDID) {
    if (msg.data[0] == 0x12 && msg.len >= 6) {         /* MULTI mode */
        uint8_t mask = msg.data[1];
        if (mask & ARM_MASK_J1) {                       /* J1 */
            int16_t v = (int16_t)(msg.data[2] | (msg.data[3] << 8));
            g_arm_state.j1_target = (float)v / 10.0f;
        }
        if (mask & ARM_MASK_J2) {                       /* J2 */
            int16_t v = (int16_t)(msg.data[4] | (msg.data[5] << 8));
            g_arm_state.j2_target = (float)v / 10.0f;
        }
        if (mask & ARM_MASK_GRIPPER) {                  /* Gripper */
            uint16_t range = g_arm_state.gripper_closed_pulse
                           - g_arm_state.gripper_open_pulse;
            g_arm_state.gripper_target = g_arm_state.gripper_open_pulse
                                       + (uint16_t)msg.data[6] * range / 200;
        }
        if (mask & ARM_MASK_J0) {                       /* J0(预留) */
            g_arm_state.j0_target = (int8_t)msg.data[7];
        }
        g_arm_state.servo_active = 1;
    }
    else if (msg.len >= 4) {                            /* LEGACY mode */
        uint8_t joint_id = msg.data[1];
        int16_t value = (int16_t)(msg.data[2] | (msg.data[3] << 8));
        g_arm_state.servo_active = 1;
        switch (joint_id) {
            case 1: g_arm_state.j1_target = (float)value / 10.0f; break;
            case 2: g_arm_state.j2_target = (float)value / 10.0f; break;
            case 3: g_arm_state.gripper_target = (uint16_t)value; break;
        }
    }
}
```

### 3.4 夹爪编码说明

| 模式 | data 位置 | 类型 | 范围 | 映射 |
|:----:|:---------:|:----:|:----:|:----|
| LEGACY | data[2:3] | int16 | 1000~5000 | 直接 TIM4 CCR 值 |
| MULTI  | data[6]   | uint8 | 0~200 | `CCR = open + val × (close-open) / 200` |

---

## 4. 0x131 — 速度设置（每关节独立）

替代当前 `0x430 0x02` 统一设速方式。

```
Byte 0: joint_id        (1=J1, 2=J2, 0xFF=ALL)
Byte 1: 保留 (填0)
Byte 2-3: speed (uint16 LE, 0.1°/s, 范围 0~3600)
Byte 4-7: 保留 (填0)
```

示例 — J1 速度设为 300°/s：
```
0x131: 01 00 B8 0B 00 00 00 00
       │     ↑____│
      J1    3000 = 300.0°/s
```

示例 — 两关节统一速度 100°/s：
```
0x131: FF 00 E8 03 00 00 00 00
       │     ↑____│
      ALL   1000 = 100.0°/s
```

---

## 5. 0x230 — 状态查询

```
Byte 0: query_type
        0x01 = QUERY_STATUS   → 触发立即发送一帧 0x330
        0x04 = LOG_START      → 开启终端 printf 调试输出
        0x05 = LOG_STOP       → 关闭终端 printf 调试输出
Byte 1-7: 保留 (填0)
```

---

## 6. 0x330 — 状态上报（TX，50ms 周期）

由 `Arm_State_Task` 每 50ms 发送。

```
Byte 0-1: J1 raw angle (uint16 LE, 0~16383, MT6701 14bit)
Byte 2-3: J2 raw angle (uint16 LE, 0~16383)
Byte 4:   J1 当前角度 (int8, ±127°, 1°精度)
Byte 5:   J2 当前角度 (int8, ±127°)
Byte 6:   夹爪位置 (uint8, 0~200 映射到 0~100%)
Byte 7:   标志位 (Flags)
          bit0: SERVO_ACTIVE   (1=已激活，可输出)
          bit1: J1_IN_RANGE    (|target - current| < 2°)
          bit2: J2_IN_RANGE
          bit3: SYSTEM_LOCKED  (1=输出被 0x430 LOCK 禁用)
          bit4-7: 保留
```

### 与 step_by_step.md 原方案的差异

step_by_step.md Phase 5 的 0x330 格式是为 J0 设计的（`data[0:1]=J0 speed`），本方案改为优先上报
J1/J2 的实际角度和状态：

| 字段 | 原方案 | 本方案（推荐） |
|:----|:------|:-------------|
| data[0:1] | J0 速度 | **J1 raw angle** |
| data[2:3] | J1 raw angle | **J2 raw angle** |
| data[4:5] | J2 raw angle | **J1/J2 压缩角度 (int8)** |
| data[6] | Gripper uint8 | Gripper uint8 (0~200) |
| data[7] | J0 stall/J1/J2 in_range flags | 通用 Flags |

**理由**：J0 尚未实现，当前优先上报 MT6701 有效数据。J0 实现后可增加 CAN ID 或格式修订。

---

## 7. 0x430 — 配置命令（扩展）

扩充原有 0x430 功能：

| data[0] | 命令 | 参数 | 说明 |
|:-------:|:----|:-----|:----|
| 0x01 | HOME（回中） | 无 | J1/J2→0°, Gripper→打开 |
| 0x02 | SET_SPEED（统一设速） | data[1:2]=int16, 0.1°/s | J1+J2 统一速度（旧协议） |
| **0x03** | **LOCK（加锁）** | 无 | `system_locked=1`，舵机输出禁用 |
| **0x04** | **UNLOCK（解锁）** | 无 | `system_locked=0`，恢复输出 |
| **0x05** | **SET_LIMITS（设限位）** | data[1:2]=J1 max, data[3:4]=J2 max, data[5:6]=min (int16, 0.1°) | J1/J2 ±限位，同一min值 |
| **0x06** | **SET_GRIPPER_LIMITS** | data[1:2]=open pulse, data[3:4]=close pulse (uint16) | 夹爪行程标定 |

---

## 8. 软件滤波（白名单）

参考 `3_MCLM_t2` 的 `App/services/can_filter.c` 模式，增加第二层软件滤波。

### 白名单表

```c
static const CAN_Filter_Entry_t s_arm_whitelist[] = {
    { CAN_ARM_CMD_STDID,    {0x11, 0x12, 0xFF} },               // 0x130
    { CAN_ARM_VEL_STDID,    {0x01, 0x02, 0xFF, 0xFF} },         // 0x131
    { CAN_ARM_QUERY_STDID,  {0x01, 0x04, 0x05, 0xFF} },         // 0x230
    { CAN_ARM_CONFIG_STDID, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFF} }, // 0x430
};
```

白名单校验在 `CAN_Rx_Task` 收到消息后立即执行，不符合的组合静默丢弃。

---

## 9. 文件变更清单

### 9.1 新建文件

| # | 文件路径 | 内容 | 预计行数 |
|:-:|:---------|:-----|:--------:|
| 1 | `App/config/app_config.h` | CAN ID 宏定义、协议常量、arm_mask、硬件参数 | ~60 |
| 2 | `App/services/can_filter.h` | 白名单 API 声明、CAN_Filter_Entry_t 类型 | ~30 |
| 3 | `App/services/can_filter.c` | 白名单表实现、CAN_Filter_Accept() | ~80 |
| 4 | `App/tasks/arm_state_task.c` | 50ms 周期 0x330 状态上报 | ~60 |

### 9.2 修改文件

| # | 文件路径 | 改动内容 |
|:-:|:---------|:---------|
| 5 | `App/config/app_globals.h` | ArmState_t 增加 limits/flags/locked/jointCmdQueue/g_debug_log |
| 6 | `Core/Src/freertos.c` | 创建 jointCmdQueue；重写 CAN_Rx_Task（白名单+0x130多关节+0x131+0x230+0x430扩展）；实现 Arm_State_Task 函数体 |
| 7 | `CMakeLists.txt` | 添加 `can_filter.c` `arm_state_task.c` 源文件；添加 `App/services` `App/tasks` include 路径 |

### 9.3 不变文件

`Core/Src/can.c` · `Core/Inc/can.h` · `Core/Src/stm32f1xx_it.c` · `App/drivers/servo.c` ·
`App/drivers/servo.h` · `App/drivers/mt6701.c` · `App/drivers/mt6701.h` · `Core/Src/tim.c`

---

## 10. 实施步骤

共 7 步，按顺序执行，每步可独立验证。

| 步骤 | 内容 | 验证方式 |
|:----:|:-----|:---------|
| **1** | 创建 `app_config.h`，扩展 `app_globals.h`（新增结构体字段、extern） | ✅ 编译通过，现有功能不变 |
| **2** | 创建 `can_filter.h/c`，插入白名单调用 | ✅ 0x130/0x430 正常，`0x999` 被拒 |
| **3** | 实现 0x130 MULTI 多关节解析 + 0x131 速度设置 | ✅ 一帧同时控 J1+J2 同步 |
| **4** | 实现 `Arm_State_Task`（0x330 TX）+ 0x230 查询 | ✅ 20Hz 状态帧，查询触即时响应 |
| **5** | 扩展 0x430（LOCK/UNLOCK/SET_LIMITS） | ✅ LOCK 禁用输出，限位裁剪角度 |
| **6** | 任务函数抽到 `App/tasks/` 独立文件 | ✅ 编译通过，功能不变 |
| **7** | 联调测试矩阵 | ✅ 全部 Case 通过 |

推荐先做**步骤 1~3**，核心多关节控制即可工作。

---

## 11. 联调测试矩阵

| 测试场景 | CAN 帧 (ID + data) | 预期结果 |
|:---------|:--------------------|:---------|
| LEGACY J1→90° | `0x130 \| 11 01 84 03 00 00 00 00` | J1 平滑移到 90° |
| LEGACY J2→90° | `0x130 \| 11 02 5A 00 00 00 00 00` | J2 平滑移到 90° |
| LEGACY 夹爪关闭 | `0x130 \| 11 03 E8 03 00 00 00 00` | 夹爪闭合 (CCR=1000) |
| **MULTI J1+J2** | `0x130 \| 12 06 84 03 2C 01 00 00` | **J1=90°, J2=30° 同时运动** |
| MULTI 全关节 | `0x130 \| 12 0E 84 03 2C 01 64 00` | J1=90°, J2=30°, Gripper=50% |
| 速度 J1 独立 | `0x131 \| 01 00 B8 0B 00 00 00 00` | J1 速度 = 300°/s |
| 速度统一 | `0x131 \| FF 00 90 01 00 00 00 00` | J1+J2 速度 = 100°/s |
| 查询状态 | `0x230 \| 01 00 00 00 00 00 00 00` | 即刻发送一帧 0x330 |
| 加锁 | `0x430 \| 03 00 00 00 00 00 00 00` | 舵机输出停止 |
| 解锁 | `0x430 \| 04 00 00 00 00 00 00 00` | 舵机恢复输出 |
| 设限位 | `0x430 \| 05 58 1B 58 1B 58 1B 00` | J1 max=70°, J2 max=70°, min=-70° |
| 回零 | `0x430 \| 01 00 00 00 00 00 00 00` | J1/J2→0° |
| 白名单拒收 | `0x999 \| ...` | 静默丢弃，无响应 |

---

## 12. 边缘情况与注意事项

1. **CAN TX mailbox 满**: `HAL_CAN_AddTxMessage()` 返回 `HAL_BUSY` 时丢弃状态帧，不重试
2. **0x12 兼容**: 旧代码忽略 `data[0]`，新代码用 `0x12` 标识多关节模式，无冲突风险
3. **夹爪类型不匹配**: LEGACY 用 data[2:3] int16，MULTI 用 data[6] uint8，两路径互斥
4. **MT6701 首次读零**: 上电后首帧 0x330 raw=0；Servo_Task 跑一次后才有效
5. **多任务共享 g_arm_state**: float 在 Cortex-M3 上 32bit 对齐读写是原子的，暂无需 mutex
