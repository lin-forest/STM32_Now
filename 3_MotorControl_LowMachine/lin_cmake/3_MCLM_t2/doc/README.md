# 3_MCLM_t2 — 文档目录

> 双电机 PID 闭环控制器项目（STM32F103C8T6, FreeRTOS, CAN, TB6612/IBT-4）
> 最后更新：2026-06-30

---

## 文档总览（35+ 篇）

| ⚓ | 分类 | 文档数 | 子目录 |
|:--:|:-----|:------:|:------:|
| 📐 | **架构设计** | 3 | `./`, `all/` |
| ⚙️ | **配置与参数** | 1 | `./` |
| 🐛 | **修复记录** | 12 | `Fix/`, `all/` |
| 🔍 | **审查与 AI 分析** | 7 | `all/claude/`, `all/gemini/`, `CanFilter/` |
| 📊 | **数据分析** | 2 | `ai_session/`, `csv_data/` |
| 📝 | **演进规划** | 3 | `all/` |
| 🧩 | **功能扩展** | 1 | `Function/` |
| 📜 | **历史版本** | 1 | `history_version/` |

---

## 📐 架构设计

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all/control.md](all/control.md) | **电机控制完整数据流** — 任务总览（8 任务）、队列与同步、核心数据结构 `Motor_t`、上行/下行数据流、PID 控制器详解、速度换算链路、硬件资源汇总、Anti-Windup/换向清积分等设计决策 | 2026-04-26 |
| [all/trae.md](all/trae.md) | **项目代码审查与数据流分析** — 项目结构概览、核心数据流（上行控制流/下行反馈流/闭环控制）、各模块接口说明、RTOS 内核对象 | 2026-04-26 |
| [all/result.md](all/result.md) | **调试数据解析记录** — CAN 0x201/0x301 帧解析、串口数据格式分析、速度反馈失效诊断（data1/data2/data3）、PWM 饱和问题定位、claude 介入前后对比 | 2026-04-26 |

---

## ⚙️ 配置与参数

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all_motor.md](all_motor.md) | **电机参数清单** — 3 种预设（DEFAULT/NEW/2IBT4）、选择机制（`MOTOR_CFG_SET`）、SPEED_TICKS_MAX 公式、电机1（TB6612/IBT-4）引脚/PID/编码器、电机2（IBT-4）TIM 资源分配、IBT-4 控制逻辑、CAN 通信参数（ID_GROUP/波特率/命令定义）、运行时结构体 `Motor_t` / `flags`、速度映射公式、快速定位 grep 命令 | 2026-06-09 |

---

## 🐛 修复记录

### Fix/ — 修复系列

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [Fix/TwoMotor.md](Fix/TwoMotor.md) | **双电机扩展计划** — Step 1~9 详细实施：`MOTOR_COUNT=2`、`CommandMsg_t.motor_id`、双队列 `MotorQueue0/1`、编码器参数化、CAN RX `motor_id` 路由 | 2026-04-19 |
| [Fix/TwoMotor_result.md](Fix/TwoMotor_result.md) | **双电机扩展实施结果** — 实际 CubeMX 命名差异、各文件改动明细（`app_globals.h`/`command.h`/`can.c`/`command_task.c`/`freertos.c`）、数据流图、无需改动文件清单 | 2026-04-20 |
| [Fix/fix1_cc.md](Fix/fix1_cc.md) | **Fix1 PID 震荡修复** — 根因链路（PWM_MAX=999、极性传错、积分限幅 500→5.66、Anti-windup、TIM2 编码器滤波/4x 模式）。6 项修复明细，从硬件层到算法层 | 2026-04-19 |
| [Fix/fix1_result.md](Fix/fix1_result.md) | **Fix1 修复日志** — 修复迭代过程：PWM 顶限日志、迭代一引入反转 Bug（极性参数）、迭代三编码器滤波/Fix（TI12 模式） | 2026-04-19 |
| [Fix/fix2_MotorDma.md](Fix/fix2_MotorDma.md) | **Fix2 Logger DMA 冲突修复方案** — 方案对比（A 合并单帧/B 双缓冲/C 队列+信号量★推荐）。方案 C 完整设计：`LogMotorData_t` 队列、信号量保护 DMA、双路电机数据解耦 | 2026-04-27 |
| [Fix/fix2_result.md](Fix/fix2_result.md) | **Fix2 实施结果** — 方案 C 落地：`LogQueueHandle`/`uart1_dma_semHandle`、encoder_task 双路投递、logger_task 出队+格式化+DMA、TxCpltCallback 信号量释放 | 2026-04-27 |
| [Fix/fix3_两电机反馈不一致.md](Fix/fix3_两电机反馈不一致.md) | **Fix3 两路反馈不一致修复** — 根因：`osSemaphoreAcquire` 写在 `snprintf` 之后导致 DMA 覆盖。修复：Acquire→snprintf→DMA 顺序修正 + 双路合并输出（方案A） | 2026-05-16 |
| [Fix/fix4_增加了编码器累计输出.md](Fix/fix4_增加了编码器累计输出.md) | **Fix4 编码器累计输出** — `Motor_t.accumulated_ticks` 新增、`LogMotorData_t` 扩展、CSV 格式 7→9 列、buffer 64→128 字节 | 2026-05-16 |
| [Fix/fix5_DC555motor.md](Fix/fix5_DC555motor.md) | **Fix5 DC555 电机的实际运行数据** — 双 IBT-4 实测定转验证，两路编码器数据 | 2026-06-07 |
| [Fix/fix6_cancelat8236.md](Fix/fix6_cancelat8236.md) | **Fix6 废弃 AT8236 驱动** — 删除 3 个文件 + 清理 7 处引用，简化维护 | 2026-06-09 |
| [Fix/fix0609_2ibt4.md](Fix/fix0609_2ibt4.md) | **Fix0609 双 IBT-4 预设新增+Bug 修复** — 新增 `MOTOR_CFG_2IBT4` 预设、`app_motor_cfg_2ibt4.h` 新建。关键 Bug：「默认指针 `&g_motors[1]` 导致转向电机无响应」修复。三大预设引脚/TIM 对比、IBT-4 驱动详解（BTS7960 芯片）、数据流追踪 | 2026-06-09 |
| [Fix/MainControl_can_plan.md](Fix/MainControl_can_plan.md) | **MainControl CAN 增强计划** — 堵转/饱和检测、状态帧格式变更（current 放首位、accumulated_ticks/target_int8/flags）、主动上报策略（50ms 超时） | 2026-05-18 |
| [Fix/MainControl_can_result.md](Fix/MainControl_can_result.md) | **MainControl CAN 增强结果** — `send_motor_status()` 统一函数、50ms 主动上报、TB6612_DC_Task 嵌入 stall/saturation 检测、涵盖文件清单 | 2026-05-18 |

### all/ — 问题修复

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all/fix_dp_CanID.md](all/fix_dp_CanID.md) | **CAN ID 组选择宏** — `#define CAN_ID_GROUP 1/2` 替代手动注释切换，`#if/#elif` 条件编译 | 2026-04-24 |
| [all/fix_MotorReversing.md](all/fix_MotorReversing.md) | **CAN 倒车控制修复** — Bug1: `(int8_t)rxData[1]` 符号丢失；Bug2: tb6612_DC_task 未处理 `CMD_REVERSE`；缺失: `CAN_CMD_REVERSE_BYTE` case。含修复后数据流 | 2026-04-24 |
| [all/result_ack.md](all/result_ack.md) | **反馈环路修复计划** — 三阶段 11 个修复（Fix1~11），涵盖精度/积分饱和/CAN 命令不灵敏/DMA TX 中断。当前 PID 参数：`Kp=1.5, Ki=0.5, integral_limit=10` | 2026-04-24 |

---

## 🔍 审查与 AI 分析

### all/claude/

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all/claude/CLAUDE.md](all/claude/CLAUDE.md) | **三步架构跃迁规划** — Step1 电机实例化 (`Motor_t motors[]`) → Step2 驱动抽象 (`MotorDriver_t` 接口) → Step3 通信抽象 (`MotorCmd_t` 统一接口)。从"能跑"到"机器人分布式执行节点" | — |
| [all/claude/Md2claude.md](all/claude/Md2claude.md) | **工程级模板** — 重构后工程结构、核心抽象层（`Motor_t`/`MotorDriver_t`/`MotorCmd_t`）、CAN 协议层 `can_service`、多电机 `Motor_Task` 控制循环、TB6612 驱动实现、CAN 状态 tx 任务 | — |
| [all/claude/result_claude.md](all/claude/result_claude.md) | **CommandQueue→MotorQueue 设计解析** — 命令分发器模式的优缺点、关注点分离、可扩展性、未来优化方向（绕过中间环节直接投递） | — |

### all/gemini/

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all/gemini/Tgemini.md](all/gemini/Tgemini.md) | **Gemini 深度源码审查报告** — App 目录全部文件审计：命令解析引擎、Logger 非阻塞 IO、PID 算法、TB6612 驱动层、RTOS 任务周期逻辑、数据全链路流转分析、变量生命周期表 | 2026-04-15 |
| [all/gemini/result1_gemini.md](all/gemini/result1_gemini.md) | **实例化升级总结报告** — v1.0 全局单例 → v2.0 对象实例化对比、已完成修改（数据结构/驱动/任务/算法稳定性）、待完成事项 | — |
| [all/gemini/result_gemini_.md](all/gemini/result_gemini_.md) | **编译报错日志** — `fabsf` 隐式声明（需 `#include <math.h>`）、`%d` 格式 `long int` 不匹配 | 2026-04-15 |

### CanFilter/

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [CanFilter/plan_dp_filter.md](CanFilter/plan_dp_filter.md) | **CAN 指令过滤设计方案** — 三级过滤架构（硬件+软件+白名单）、表格驱动白名单 API、`can.c` 回调精简 ~70→20 行 | 2026-05-08 |
| [CanFilter/result_dp_filter.md](CanFilter/result_dp_filter.md) | **CAN 过滤实施结果** — `can_filter.h/c` 新建、零改动 `command_task.c`。实时修复：`0x101` 全车停止需要 `0x11` 命令字节 | 2026-05-08 |

---

## 📊 数据分析

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [ai_session/can_data_analyze.md](ai_session/can_data_analyze.md) | **CAN 数据分析完整记录** — 23974 帧录制数据验证新格式：50ms±2ms 主动上报、动电机堵转事件（flags=0x03 持续 3.8s）、转向电机正常运行（0→50→80→-30 全行程）、自由停车 1.8 秒、字节序/时间戳确认 | 2026-05-18 |
| [csv_data/Can_MainControl_t1.csv](csv_data/Can_MainControl_t1.csv) | CAN 总线录制数据集 — 23974 帧，1.3MB，覆盖完整的控制+反馈交互 | 2026-05-18 |

---

## 📝 演进规划

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [all/goal.md](all/goal.md) | **架构升级三步路线** — Step1 全局变量→实例化、Step2 驱动抽象、Step3 通信抽象 | — |
| [all/goal2.md](all/goal2.md) | **机器人执行节点模板** — 完整的 `MotorCmd_t`/`MotorDriver_t`/`Motor_t` 三层架构，CAN 协议骨架、多电机控制任务模板 | — |

---

## 🧩 功能扩展

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [Function/Func1_planMt6701.md](Function/Func1_planMt6701.md) | **MT6701 角度闭环控制计划** — 级联 PID 架构（位置外环 20Hz + 速度内环 100Hz）、SPI1 重映射至 PB3/PB4、传动比映射函数、角度数据 int32_t x10 定标、误差驱动法 PID、磁铁检测确认 | 2026-06-03 |

---

## 📜 历史版本

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [history_version/can.md](history_version/can.md) | **CAN 数据流文档（旧版）** — 原始内联协议解析架构，`0x125/0x126` 系列 ID、纯轮询模式、TX 帧旧格式（`[0-1]=target + [6-7]=reserved`） | — |

---

## 文档关系图

```
架构设计
  ├── all/control.md, all/trae.md  ← 完整数据流 + 任务拓扑
  └── all_motor.md                 ← 电机参数配置中枢

修复记录 ← 顺序最接近实施时间
  ├── TwoMotor.md → TwoMotor_result.md    ← 双电机化
  ├── fix1_cc.md + fix1_result.md         ← 首次 PID 震荡修复（6 项）
  ├── fix2_MotorDma.md → fix2_result.md   ← Logger DMA 冲突
  ├── fix3_两电机反馈不一致.md             ← 信号量顺序修正
  ├── fix4_编码器累计输出.md               ← accumulated_ticks
  ├── fix5_DC555motor.md                  ← 实测定转
  ├── fix6_cancelat8236.md                ← 清理
  ├── fix0609_2ibt4.md                    ← 双 IBT-4 + 指针 Bug
  ├── MainControl_can_plan → result       ← CAN 主动上报
  ├── fix_dp_CanID.md                     ← CAN_ID_GROUP 宏
  ├── fix_MotorReversing.md               ← 倒车修复
  └── result_ack.md                       ← 汇总 11 个 Fix

审查与 AI
  ├── all/claude/    ← 架构跃迁三步 + 工程模板
  ├── all/gemini/    ← 深度源码审查 + 实例化总结
  └── CanFilter/     ← 表格驱动白名单

数据分析
  └── ai_session/can_data_analyze.md  ← 23974 帧录制验证

功能扩展
  └── Function/Func1_planMt6701.md    ← MT6701 角度闭环
```

---

## 建议阅读顺序

### 新项目成员
1. **[all/control.md](all/control.md)** — 当前架构全貌
2. **[all_motor.md](all_motor.md)** — 电机配置系统
3. **[all/trae.md](all/trae.md)** — 项目代码结构

### 了解修复历史
4. **[Fix/fix1_cc.md](Fix/fix1_cc.md)** — 首次 PID 震荡（6 项根因）
5. **[Fix/fix0609_2ibt4.md](Fix/fix0609_2ibt4.md)** — 双 IBT-4 + Bug（指针错误）
6. **[Fix/MainControl_can_result.md](Fix/MainControl_can_result.md)** — CAN 主动上报
7. **[Fix/TwoMotor_result.md](Fix/TwoMotor_result.md)** — 双电机化

### 深入分析
8. **[ai_session/can_data_analyze.md](ai_session/can_data_analyze.md)** — CAN 录制验证
9. **[CanFilter/result_dp_filter.md](CanFilter/result_dp_filter.md)** — 过滤架构
10. **[Function/Func1_planMt6701.md](Function/Func1_planMt6701.md)** — MT6701 功能扩展

### 架构演进
11. **[all/claude/CLAUDE.md](all/claude/CLAUDE.md)** — 三步跃迁规划
12. **[all/claude/Md2claude.md](all/claude/Md2claude.md)** — 工程级模板
