# Fix1 修复记录

> 工程：`3_MCLM_t2`  日期：2026-04-19  
> 问题现象：速度稍微拉高后 PWM 顶限运行，伴随震荡

---

## 数据格式说明

串口日志格式：`tick, encoder_cnt, speed(ticks/10ms), target, pwm_output`

---

## 修复迭代一（引入新 Bug）

### 修改内容

| 文件 | 修改点 | 修改前 | 修改后 |
|---|---|---|---|
| `app_config.h` | `PWM_MAX` | `999` | `99` |
| `app_config.h` | `MOTOR1_PID_INTEGRAL_LIMIT` | `10.0f` | `5.66f` |
| `tb6612_DC_task.c` | `TB6612_Motor_Init()` 参数 | `MIN_PWM_OUTPUT=0, DEAD_ZONE=10` | `DEAD_ZONE=10, Polarity=0` |
| `pid.c` | 积分策略 | 简单限幅 | 增加条件积分（Anti-windup） |

### 反馈数据（fix1_result.md）分析

```
[16:40:31.931] 67013,35443,-35,32.0,99
[16:40:32.039] ...  ,35211,  0,32.0,99   ← 短暂归零
[16:40:32.308] ...  ,35191,-14,32.0,99   ← 再次反转
...（周期性：反转约250ms → 归零约150ms → 再反转）
```

| 观测 | 含义 |
|---|---|
| `speed` 全为负值 | 电机在**反转** |
| `encoder_cnt` 持续递减 | 电机持续向负方向旋转 |
| `pwm_output` 恒定 `99` | PWM 永远顶限 |
| `target = 32.0`（正） | 目标为正方向 |

**结论**：`error = 32 - (-35) = 67`（极大正误差）→ PID 输出最大正值 → PWM=99 → 电机反转更快 → 形成**正反馈死循环**，震荡比修复前更剧烈。

### 新引入 Bug 的定位

`Polarity=0`（正向）改变了上一轮修复中的 `Polarity=0`。  
原始代码中 `Polarity` 实际传入的是 `MOTOR1_DEAD_ZONE=10`（位置传错），虽然是"事故"，但 `10 != 0` → 极性反转效果是正确的——该电机**物理接线本身需要极性反转**才能使编码器正方向与指令正方向一致。

```
迭代一修复前: DeadZone=0(错), Polarity=10(偶然正确) → 电机正转
迭代一修复后: DeadZone=10(对), Polarity=0(错)        → 电机反转 ← 新 Bug
```

---

## 修复迭代二（当前状态）

### 修改内容

| 文件 | 行 | 修改 |
|---|---|---|
| `tb6612_DC_task.c` | 28 | `Polarity: 0 → 1`（恢复极性反转，明确标注原因） |

```c
// 修复后
TB6612_Motor_Init(..., MOTOR1_DEAD_ZONE,
                  1,   // Polarity=1: 此电机硬件接线需要极性反转
                  TB6612_MOTOR_STOP_BRAKE);
```

### 当前所有有效修复汇总

| # | 文件 | 问题 | 修复 |
|---|---|---|---|
| 1 | `app_config.h` | `PWM_MAX=999` 是 TIM3 ARR(99) 的 10 倍，任何输出都 100% 占空比 | 改为 `99` |
| 2 | `tb6612_DC_task.c` | `DeadZone` 传入 `MIN_PWM_OUTPUT=0`，`Polarity` 传入 `DEAD_ZONE=10`（位置错） | `DeadZone=MOTOR1_DEAD_ZONE(10)`, `Polarity=1` |
| 3 | `app_config.h` | `integral_limit=10` 时 `Ki×limit=176.6`，远超 `output_limit=100`，积分深度饱和 | 改为 `5.66f`（使 `Ki×limit≈100=output_limit`） |
| 4 | `pid.c` | 无 Anti-windup，输出饱和时积分继续累积，切换目标时震荡 | 增加条件积分：饱和且误差同向时锁定积分 |

---

## 根因链路图

```
PWM_MAX=999 (×10倍)
    └→ CCR 写入值 > ARR → 硬件强制 100% 占空比
        └→ 任何 speed 都全速运转
            └→ PID 永远看到大误差 → 永远输出最大
                └→ 震荡（积分无限累积加剧）

Polarity 参数位置传错 (原: DEAD_ZONE 当 Polarity 传)
    └→ 偶然正确（=1=反转）→ 电机实际正转
        └→ 迭代一错误修复为 Polarity=0
            └→ 电机反转 → 正反馈 → 震荡更剧烈
```

---

## 待观察

迭代二修复后，电机应正向旋转，PID 能正常闭环。  
若仍有轻微震荡，下一步方向：
1. 适当降低 `Ki`（当前 `17.66f` 偏大）或增大 `Kd` 抑制超调
2. 开启编码器滤波（`iir_filter`，当前已注释）
3. 检查编码器/控制任务时序是否同周期竞争（均为 `osDelay(10)`）
