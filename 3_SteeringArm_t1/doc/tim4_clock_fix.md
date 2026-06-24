# TIM4 舵机 PWM 频率配置记录

> 日期：2026-06-24
> 芯片：STM32F103C8T6
> 问题：TIM4 输出频率错误导致舵机不转/范围不足

---

## 一、问题现象

| 阶段 | PSC | 现象 |
|:---:|:---:|------|
| 初始（CubeMX 默认） | 0 | 舵机完全不动（549Hz，不认） |
| 第一次修正 | 17 | 舵机能动，但 300° 范围只走了 ~120° |
| 最终修正 | **35** | ✅ 正常 300° 全范围 |

## 二、根本原因：STM32F1 定时器时钟翻倍规则

### 规则（RM0008 §5.2）

> 如果 APB1 预分频系数 = 1，则定时器时钟 = APB1 时钟
> 如果 APB1 预分频系数 ≠ 1，则定时器时钟 = APB1 时钟 × **2**

### 本项目时钟树

```
HSE 8MHz → PLL ×9 → SYSCLK = 72MHz
  → APB1 = 72MHz / 2 = 36MHz     ← 分频 ≠ 1
  → APB1 定时器时钟 = 36MHz × 2 = 72MHz  ← 翻倍！
```

### 错误计算过程

```
我最初以为：
  TIM4 时钟 = APB1 = 36MHz ❌
  PSC=17 → ÷18 → 2MHz → tick=0.5μs
  ARR=39999 → period=40000 → 2MHz/40000=50Hz

实际是：
  TIM4 时钟 = 72MHz（翻倍）✅
  PSC=17 → ÷18 → 4MHz → tick=0.25μs  ← 算错了！
  ARR=39999 → 4MHz/40000=100Hz ❌ 不是 50Hz
  CCR=3000 → 3000×0.25μs=750μs ❌ 不是 1500μs
```

## 三、最终正确参数

配置在 CubeMX → TIM4 → Parameter Settings：

| 参数 | 值 | 计算过程 |
|:----|:---:|---------|
| Prescaler | **35** | 72MHz / (35+1) = **2MHz** |
| Counter Period | **39999** | 2MHz / (39999+1) = **50Hz** ✅ |
| 每 tick | 0.5 μs | 1 / 2MHz |
| AutoReloadPreload | Enable | 平滑更新 |

### 脉宽验证

| 角度 | 脉宽 | CCR | 计算 |
|:---:|:----:|:---:|:----|
| -150° | 500 μs | 1000 | 500 / 0.5 |
| 0°（中位） | **1500 μs** | **3000** | 1500 / 0.5 |
| +150° | 2500 μs | 5000 | 2500 / 0.5 |

## 四、检查方法

每次配置 TIM 后，通过计算验证频率：

```c
// 在 stm32f1xx_hal_timebase_tim.c 或 tim.c 中确认
// TIM4 实际计算：
//   时钟源 = (APB1 分频 ≠ 1) ? APB1×2 : APB1
//   定时器频率 = 时钟源 / (Prescaler + 1)
//   PWM 频率 = 定时器频率 / (Period + 1)
//
// 本项目：
//   时钟源 = 36MHz × 2 = 72MHz  ← APB1=36MHz, 分频≠1
//   定时器频率 = 72MHz / (35+1) = 2MHz
//   PWM 频率 = 2MHz / (39999+1) = 50Hz ✅
```

## 五、参考

- STM32F1 参考手册 RM0008 Rev20, §5.2: "If the APB1 prescaler is 1, the timers are clocked at the same frequency as APB1. Otherwise, they are clocked at twice the frequency of APB1."
- 该规则适用于 STM32F1 全系列（F103/F105/F107 等）
- F4 系列没有这个翻倍规则，APB 定时器时钟直接等于 APB 时钟
