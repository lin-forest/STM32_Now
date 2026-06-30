# 相对角度方案规划

> 文档版本: v1.0
> 创建日期: 2026-06-26
> 状态: 规划阶段 ⏳

---

## 1. 背景

当前 MT6701 反馈显示的是**绝对角度**（基于机械中位标定：J1 offset=16300, J2 offset=12200）。
需要两种相对角度能力：

1. **反馈相对角度** — printf 显示 / 状态上报用"离上电位置的偏移"而非绝对角度
2. **控制相对角度** — CAN 0x130 支持增量指令（"再 +10°"而非"到 45°"）

两者独立，可分别实现。

---

## 2. 概念区分

| 概念 | 含义 | 举例 |
|:----|:-----|:-----|
| **绝对角度** | 基于机械中位标定的真实角度 | J1=-93.6°（默认姿态） |
| **反馈相对角度** | 离上电零位的偏移 | 上电时 J1=0°，转 10° 后 J1=+10° |
| **控制相对角度** | CAN 指令为增量而非绝对值 | "再 +10°" |

---

## 3. 方案一：反馈相对角度

### 3.1 设计思路

上电瞬间记录 MT6701 读数为"零位 raw"，后续所有角度显示用 `零位 raw` 而非 `机械中位 offset` 做偏移计算。

```
上电:  raw=11950  → 记录 j1_zero_raw=11950
      printf: J1=  0 (因为相对零位偏移为 0)

运动后: raw=12405 → j1_rel = MT6701_RawToAngleX10(12405, 11950)
      printf: J1=+100 (即 +10.0°)
```

### 3.2 涉及文件

#### `App/config/app_globals.h` — ArmState_t 新增字段

```c
/* 相对角度零位（上电时记录，可用 CAN 重零） */
uint16_t j1_zero_raw;
uint16_t j2_zero_raw;
```

#### `Core/Src/freertos.c` (Servo_Task) — 首次运行时自零

```c
for(;;)
{
    /* 读 MT6701 */
    g_arm_state.j1_raw = MT6701_ReadRaw(J1_CS_GPIO_Port, J1_CS_Pin);
    g_arm_state.j2_raw = MT6701_ReadRaw(J2_CS_GPIO_Port, J2_CS_Pin);

    /* 首次运行记录零位 */
    static uint8_t zero_inited = 0;
    if (!zero_inited) {
        g_arm_state.j1_zero_raw = g_arm_state.j1_raw;
        g_arm_state.j2_zero_raw = g_arm_state.j2_raw;
        zero_inited = 1;
        printf("Zero reference set: J1=%u J2=%u\r\n",
               g_arm_state.j1_zero_raw, g_arm_state.j2_zero_raw);
    }

    /* 相对角度计算 */
    int16_t j1_rel = MT6701_RawToAngleX10(g_arm_state.j1_raw, g_arm_state.j1_zero_raw);
    int16_t j2_rel = MT6701_RawToAngleX10(g_arm_state.j2_raw, g_arm_state.j2_zero_raw);

    printf("J1=%+5d J2=%+5d\r\n", j1_rel, j2_rel);

    /* ... 舵机控制逻辑不变 ... */
    osDelay(20);
}
```

#### `0x430` 扩展 — 重零命令

```
0x430: 0x07 → 重零（当前角度设为 0° 参考点）
```

CAN_Rx_Task 中：
```c
if (msg.id == 0x430) {
    switch (msg.data[0]) {
        case 0x07:  // 重零
            g_arm_state.j1_zero_raw = g_arm_state.j1_raw;
            g_arm_state.j2_zero_raw = g_arm_state.j2_raw;
            printf("=== ZERO SET ===\r\n");
            break;
    }
}
```

### 3.3 关于 0x330 状态上报的考虑

| 选项 | printf 显示 | 0x330 上报 | 推荐？ |
|:----|:-----------|:----------|:------:|
| A | 相对 | **绝对** | **✅ 推荐** — 底盘控制器始终看到真实绝对位置 |
| B | 相对 | 相对 | 需底盘控制器知零位，增加复杂度 |

**推荐 A**：printf 看偏移方便调试，0x330 上报绝对角度保证上位机信息完整。

---

## 4. 方案二：控制相对角度

### 4.1 设计思路

在 0x130 LEGACY 模式下，用 `data[0]` 区分绝对/增量：

| data[0] | 模式 | 含义 |
|:-------:|:----|:-----|
| `0x11` | 绝对（现有） | J1 到 45° |
| `0x21` | 增量（新增） | J1 再 +10° |

### 4.2 CAN 帧格式

```
0x130 | 21 01 64 00 00 00 00 00
       │  │  ↑____│
     增量 J1  +100 = +10.0°
```

```
0x130 | 21 02 38 FF 00 00 00 00 00
             ↑____│
            -200 = -20.0°
```

### 4.3 解析实现

在 CAN_Rx_Task 0x130 分支中增加：

```c
if (msg.id == 0x130 && msg.len >= 4) {
    uint8_t cmd = msg.data[0];
    uint8_t joint_id = msg.data[1];
    int16_t value = (int16_t)(msg.data[2] | (msg.data[3] << 8));

    if (cmd == 0x21) {     // 相对/增量模式
        float delta = (float)value / 10.0f;
        g_arm_state.servo_active = 1;
        switch (joint_id) {
            case 1: g_arm_state.j1_target += delta; break;
            case 2: g_arm_state.j2_target += delta; break;
            case 3: g_arm_state.gripper_target += value; break;
        }
        printf("ARM_INC: j%d += %d → target=%.0f\r\n",
               joint_id, value, g_arm_state.j1_target);
    }
    else {                 // 绝对模式（0x11 或其他，现有逻辑）
        g_arm_state.servo_active = 1;
        switch (joint_id) {
            case 1: g_arm_state.j1_target = (float)value / 10.0f; break;
            case 2: g_arm_state.j2_target = (float)value / 10.0f; break;
            case 3: g_arm_state.gripper_target = (uint16_t)value; break;
        }
    }
}
```

### 4.4 与 MULTI 多关节模式的配合

MULTI 模式也可以支持相对控制——用 `data[0]=0x22` 标识：

```
data[0]=0x22 → MULTI 增量版本
data[1]=bitmask
data[2:3]=J1 增量 (int16, 0.1°)
data[4:5]=J2 增量 (int16, 0.1°)
data[6]=夹爪增量 (int8, 0~200)
```

---

## 5. 文件变更清单

| # | 文件 | 改动 |
|:-:|:-----|:-----|
| 1 | `App/config/app_globals.h` | ArmState_t 增加 `j1_zero_raw`, `j2_zero_raw` |
| 2 | `Core/Src/freertos.c` | Servo_Task: 自零逻辑 + 相对角度 printf；CAN_Rx_Task: 0x21 增量控制 + 0x430 0x07 重零 |

不变文件：`mt6701.c/h`（已提供 `MT6701_RawToAngleX10()` 含回绕，复用即可）

---

## 6. 实施步骤

| 步骤 | 内容 | 涉及 | 验证 |
|:----:|:-----|:-----|:-----|
| 1 | app_globals.h 加 zero_raw 字段 | app_globals.h | 编译通过 |
| 2 | Servo_Task 自零逻辑 + 相对 printf | freertos.c | 上电 J1=0, J2=0，转轴后角度从 0 变化 |
| 3 | 0x21 增量控制 | freertos.c (CAN_Rx_Task) | 连续增量帧确认目标累加 |
| 4 | 0x430 0x07 重零 | freertos.c (CAN_Rx_Task) | 运动中重零，printf 归零 |

---

## 7. 测试矩阵

| 测试 | 操作 | 预期 |
|:----|:-----|:-----|
| 上电自零 | 上电看 printf | J1=0, J2=0 |
| 手动转轴 | 手动转 J1 | J1 从 0 平滑变化 |
| 绝对控制 | `0x130 \| 11 01 84 03 ...` | J1 到 90°（绝对机械位置） |
| 增量控制 | `0x130 \| 21 01 64 00 ...` | J1 再 +10°，目标累计到 100° |
| 连续增量 | 重复 `21 01 64 00` | 每次 +10°，可累计多次 |
| 重零 | `0x430 \| 07 ...` | printf J1 归零 |
| 重零后增量 | 重零后发 `21 01 64 00` | 从新零位 +10° |
