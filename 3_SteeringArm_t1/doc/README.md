# 3_SteeringArm_t1 — 文档目录

> 机械臂控制器项目文档索引
> 最后更新：2026-06-30

---

## 文档分类

### 📐 架构与方案设计

| 文档 | 说明 | 日期 |
|:-----|:-----|:----:|
| [plan_steering_arm.md](plan_steering_arm.md) | **主架构文档** — 系统架构、硬件资源分析、引脚分配、CubeMX 配置、FreeRTOS 任务设计、CAN 协议、代码复用策略、实施计划 | 2026-06-23 |
| [can_protocol_plan.md](can_protocol_plan.md) | **CAN 协议扩展规划** — 多关节同步控制（0x12 MULTI）、独立速度设置（0x131）、状态上报（0x330）、白名单滤波、配置扩展（0x430 LOCK/LIMITS） | 2026-06-25 |
| [relative_angle_plan.md](relative_angle_plan.md) | **相对角度方案规划** — 反馈相对角度（上电位自零）、控制相对角度（0x21 增量指令）、CAN 重零命令 | 2026-06-26 |

### 📋 实施与状态

| 文档 | 说明 | 日期 |
|:-----|:-----|:----:|
| [step_by_step.md](step_by_step.md) | **逐步实施指南** — 按 Phase/Step 的实操步骤、CubeMX 点击次序、代码示例、验收标准、快速参考文件清单 | 2026-06-24 |
| [status.md](status.md) | **项目状态** — 已完成功能、FreeRTOS 任务现状、CAN 协议摘要、待实现任务清单（P0/P1）、关键注意事项 | 2026-06-24 |

### 🐛 调试记录

| 文档 | 说明 | 日期 |
|:-----|:-----|:----:|
| [debug_log.md](debug_log.md) | **调试日志** — TIM4 舵机 PWM 频率错误、平滑插值 + g_servo_active 配合问题、printf 浮点崩溃、CAN 文件行尾 CRLF | 2026-06-24 |
| [tim4_clock_fix.md](tim4_clock_fix.md) | **TIM4 时钟翻倍记录** — STM32F1 APB1 定时器时钟翻倍规则的根因分析和最终正确参数（PSC=35, ARR=39999） | 2026-06-24 |
| [verify_printf.md](verify_printf.md) | **printf 验证记录** — USART1 printf 重定向（`__io_putchar`）的代码改动和验收 | 2026-06-24 |

### 📦 memory/ — 持久化参考

| 文档 | 说明 |
|:-----|:-----|
| [memory/mt6701-calibration.md](memory/mt6701-calibration.md) | MT6701 标定参数 — J1/J2 中位偏移（16300/12200）、行程范围、回绕处理 |
| [memory/project-rename-map-error.md](memory/project-rename-map-error.md) | CubeMX 改名后 STM32 扩展 map 文件不匹配的修复 |

---

## 文档关系图

```
计划/设计 ← 规划
  ├── plan_steering_arm.md        ← 整体架构设计
  ├── can_protocol_plan.md        ← CAN 协议扩展
  └── relative_angle_plan.md      ← 相对角度方案

实施/状态 ← 执行
  ├── step_by_step.md             ← Phase 1→6 逐步指南
  └── status.md                   ← 当前进展快照

调试 ← 记录
  ├── debug_log.md                ← 串联日志
  ├── tim4_clock_fix.md           ← TIM4 专项
  └── verify_printf.md            ← printf 专项

持久化 ← 跨会话参考
  └── memory/                     ← 标定参数、历史修复
```

---

## 建议阅读顺序

1. **[plan_steering_arm.md](plan_steering_arm.md)** — 先了解整体架构和设计决策
2. **[step_by_step.md](step_by_step.md)** — 按步骤实施
3. **[status.md](status.md)** — 查看当前进展和待办
4. **[can_protocol_plan.md](can_protocol_plan.md)** — 涉及 CAN 通信时参考
5. **[relative_angle_plan.md](relative_angle_plan.md)** — 涉及角度标定时参考
6. **[debug_log.md](debug_log.md)** — 遇到问题时查找踩坑记录
