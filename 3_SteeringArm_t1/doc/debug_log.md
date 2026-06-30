# 调试记录

> 日期：2026-06-24
> 项目：3_SteeringArm_t1

---

## 1. TIM4 舵机 PWM 频率错误

### 现象
舵机不转 / 只转 ~60° / 全行程但频率错

### 根因
STM32F1 的 APB1 定时器时钟翻倍规则（RM0008 §5.2）：

```
APB1 = 72MHz/2 = 36MHz
APB1 分频 ≠ 1 → 定时器时钟 = 36MHz × 2 = 72MHz  ← 翻倍！
```

我最初按 36MHz 算的，PSC=17 → 72/18=4MHz → tick=0.25μs → 50Hz 算错了。

### 解决
PSC=35: 72/36=2MHz → tick=0.5μs → 50Hz ✅

### 文件
`doc/tim4_clock_fix.md`

---

## 2. 舵机平滑插值 + g_servo_active 配合问题

### 现象
首次激活时，`j1_current=0`（零初始化），第一帧 `Servo_SetAngle(step)` 输出接近 0°，舵机从物理位置弹向中心。

### 错误修复尝试
加了两行 `j1_current = j1_target` → diff=0 → 调速失效（平滑插值永不执行）

### 当前状态
已回退。恢复原始平滑逻辑：从 0° 开始向目标平滑插值。第一跳不可避免，需等 MT6701 反馈才能解决。

### 关键代码
```c
// CAN_Rx_Task 中 0x130 处理
g_servo_active = 1;                 // 激活
g_arm_state.j1_target = value/10;   // 设目标（不碰 current）

// Servo_Task 中 20ms 循环
diff = j1_target - j1_current;
step = speed_dps * 0.02;
if (step > |diff|) step = |diff|;
j1_current += (diff > 0) ? step : -step;
Servo_SetAngle(channel, j1_current);  // 平滑输出
```

---

## 3. printf 浮点崩溃

### 现象
`printf("%+.0f°")` 导致系统异常

### 根因
`_printf_float` 未链接到 newlib-nano

### 解决
改用整数打印：`printf("J1=%+d", (int)(current * 10))`

---

## 4. CAN 文件行尾 CRLF

### 现象
VS Code 和 CubeMX 生成的文件使用 CRLF 行尾，导致 `Edit` 工具匹配失败。

### 解决
用 `sed` 或 Python 处理，或先 `sed -i 's/\r$//'` 转 LF。

---

## 5. 待解决

| 问题 | 方案 | 依赖 |
|:----|:----|:----:|
| 首次激活弹向 0° | 上电读 MT6701 → 初始化 j1_current | MT6701 接线 |
| 舵机误差 | 闭环控制（MT6701 反馈 + PID） | MT6701 接线 |
| 无 CAN 状态上报 | 实现 Arm_State_Task | — |
