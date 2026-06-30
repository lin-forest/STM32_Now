# 5_ChassisController_t1 — 文档目录

> 底盘控制器项目（UART↔CAN 网关 → 分布式控制节点演进）
> 最后更新：2026-06-30

---

## 文档总览（27 篇）

| ⚓ | 分类 | 文档数 | 子目录 |
|:--:|:-----|:------:|:------:|
| 📐 | 架构设计与分析 | 3 | `./` |
| 📋 | 演进规划 | 4 | `other/`, `plan/` |
| 🐛 | 代码审查与修复 | 8 | `DesignComparison/`, `DesignComparison_2/` |
| 📝 | 改造记录 | 1 | `other/` |
| 📦 | 其他 | — | — |

---

## 📐 架构设计与分析（根目录）

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [Now_all.md](Now_all.md) | **当前架构与数据流分析** — Phase 1 总览：分层架构、5 个 FreeRTOS 任务详解、完整数据流图（上行 UART→CAN / 下行 CAN→SystemState）、任务间通信与同步机制、CAN 命令层分析、关键设计决策 | 2026-06-18 |
| [chassis_model_analysis.md](chassis_model_analysis.md) | **底盘构型分析与网关抽象层设计** — 两种底盘构型（舵轮/四全向轮）、四层数据处理（解码→融合→推理→推导）、运动学接口设计、CHASSIS_STATE/EVENT/ODOM 消息格式、当前实现状态与演进路线 | 2026-06-22 |
| [result.md](result.md) | **项目目标完成状态** — 基础功能（四任务架构/自定义 UART 协议/UART↔CAN 转换/错误处理）+ 代码审查优化（环形缓冲区原子性/UART 多任务互斥/事件驱动/代码规范） | 2026-05-07 |

---

## 📋 演进规划

### `other/`

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [other/trae.md](other/trae.md) | **架构与数据流原始设计文档** — 三层架构（HAL/OS/App）、四核心任务详解、环形缓冲区/消息队列/互斥锁/事件标志等同步组件、UART 协议格式、诊断调试机制 | 2026-05-07 |
| [other/goal.md](other/goal.md) | **后续工作与演进方向** — 近期验证要点 4 项、中长期演进 4 方向（协议功能增强/性能资源优化/健壮性提升/接口扩展） | 2026-05-07 |
| [other/goal2_ToMaster.md](other/goal2_ToMaster.md) | **网关→下层主控架构演进设计讨论** — 分析现有"数据转发系统"不足，提出"系统状态层+CommandProcess+IMU任务+ControlTask"7 任务架构升级方案 | 2026-05-07 |
| [other/plan2_ToMaster.md](other/plan2_ToMaster.md) | （空文件，预留）演进实施规划 | — |

### `plan/`

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [plan/plan_ToMaster.md](plan/plan_ToMaster.md) | **网关→下层主控演进计划** — 完整 6 Phase 实施：Phase 0 对齐→Phase 1 SystemState+任务重构→Phase 2 CAN 协议集成→Phase 3 IMU 集成→Phase 4 控制任务→Phase 5 调试诊断，含每步涉及文件清单和验证方案 | 2026-06-11 |
| [plan/chassis_dual_type_support.md](plan/chassis_dual_type_support.md) | **底盘构型支持实现计划** — 4 Phase 实施：A 基础设施（底盘选型配置/CAN ID 映射表/运动学模块）、B 重构现有任务用查表、C 加底盘级命令和状态、D 切换构型。含表驱动映射设计、运动学公式、内存影响预估 | 2026-06-18 |

---

## 🐛 代码审查与修复记录

### DesignComparison — 第一轮审查（Phase 1 修复）

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [DesignComparison/deepseek_t1.md](DesignComparison/deepseek_t1.md) | **当前架构设计审查** — 6 大问题：优先级反转风险、TOCTOU 竞态、cmd 未透传 CAN、下行纯文本格式、关键 API 返回值未检查、DMA 死锁、App-CubeMX 耦合 | 2026-05-07 |
| [DesignComparison/Q1_dp_t1_comparsion.md](DesignComparison/Q1_dp_t1_comparsion.md) | **CAN 数据流文档** — CAN ID 定义（0x123/0x124/0x323/0x324 等）、RX/TX 数据流、协议解析（中断回调内联）、LOG 回传、ACK 旁路流、路由函数、队列拓扑 | 2026-05-08 |
| [DesignComparison/A1_dp_t1.md](DesignComparison/A1_dp_t1.md) | **cmd 未透传 CAN 问题完整解读** — 完整数据流追溯结论："不是 bug，是注释写错了"。两套命令编码对比（UART cmd vs CAN data[0]）、工程历史背景、审查流程建议 | 2026-05-08 |
| [DesignComparison/fix2_队列满检查.md](DesignComparison/fix2_队列满检查.md) | **Fix2 队列满检查** — 3 处 `osMessageQueuePut` 加返回值检查 + 丢帧计数器递增（`uartToCanQueue_drop_cnt` / `canRxQueue_drop_cnt`） | 2026-05-08 |
| [DesignComparison/fix3_信号量死锁.md](DesignComparison/fix3_信号量死锁.md) | **Fix3 DMA 信号量死锁修复** — `uart1_send()` 加 DMA 返回值检查 + `osSemaphoreAcquire` 1000ms 超时保护，防止互斥锁永占导致全系统死锁 | 2026-05-08 |
| [DesignComparison/result_dp_t1.md](DesignComparison/result_dp_t1.md) | **Phase 1 修复计划总览** — 3 项修复：Fix1 cmd 透传（计划改数据通路）、Fix2 队列检查、Fix3 死锁保护，含改动量和实施顺序 | 2026-05-07 |

### DesignComparison_2 — 第二轮审查（反馈设计修复）

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [DesignComparison_2/Q2_comparison_t1.md](DesignComparison_2/Q2_comparison_t1.md) | **反馈问题分析** — P0: PC 端无行缓冲致 ID 错位、P1: "Sending..." 打印在发送前、P2: 无 CAN TX 完成回调、P3: 诊断日志竞争。含 Python 脚本（curses 控制+串口通信）和 RX 日志截图 | 2026-05-08 |
| [DesignComparison_2/D2fix1_plan.md](DesignComparison_2/D2fix1_plan.md) | **反馈设计修复计划** — Fix1 P0: PC 端行缓冲（`io.BytesIO`）、Fix2 P1: 打印顺序语义修正、Fix3 P2: CAN TX 完成回调、Fix4 P3: 降低诊断频率 | 2026-05-08 |
| [DesignComparison_2/D2result1.md](DesignComparison_2/D2result1.md) | **反馈设计修复结果** — 实测 Done 从 1→12 递增，ID 错位消除。踩坑记录：`HAL_CAN_TxCpltCallback` vs `HAL_CAN_TxMailboxNCompleteCallback` 差异（ F1 vs F4/H7）、NVIC 使能 ≠ 外设中断使能、`__HAL_CAN_ENABLE_IT` 必要性 | 2026-05-08 |

---

## 📝 改造记录

| 文件 | 说明 | 日期 |
|:-----|:-----|:----:|
| [other/fix1_UartToDma.md](other/fix1_UartToDma.md) | **UART TX 阻塞→DMA 改造** — `HAL_UART_Transmit`(阻塞)改为 `HAL_UART_Transmit_DMA` + 信号量同步，含同步机制选择论证（Mutex vs Semaphore）、信号量初始计数必须为 0、DMA 缓冲区为何必须 static | 2026-05-07 |

---

## 文档关系图

```
架构设计
  ├── Now_all.md                    ← Phase 1 数据流总览
  ├── chassis_model_analysis.md     ← 底盘构型 + 运动学抽象
  └── result.md                     ← 已完成目标

演进规划
  ├── trae.md, goal.md              ← 原始架构 + 后续方向
  ├── goal2_ToMaster.md             ← 架构升级思想（首次讨论）
  ├── plan/plan_ToMaster.md         ← 详细 6 Phase 实施计划
  └── plan/chassis_dual_type_support.md ← 底盘构型支持

审查修复 ← 第一轮（Phase 1 基础修复）
  ├── deepseek_t1.md                ← 6 项问题审查
  ├── Q1_dp_t1_comparsion.md / A1_dp_t1.md  ← 数据流分析/cmd 澄清
  ├── fix2_队列满检查.md, fix3_信号量死锁.md   ← 具体修复
  └── result_dp_t1.md               ← 修复计划

审查修复 ← 第二轮（反馈设计修复）
  ├── Q2_comparison_t1.md           ← 4 项反馈问题
  ├── D2fix1_plan.md → D2result1.md  ← 计划→结果（含踩坑记录）

改造记录
  └── fix1_UartToDma.md             ← DMA 发送改造
```

---

## 建议阅读顺序

### 新项目成员
1. **[Now_all.md](Now_all.md)** — 当前架构全貌
2. **[other/trae.md](other/trae.md)** — 原始设计意图
3. **[chassis_model_analysis.md](chassis_model_analysis.md)** — 目标架构设计

### 了解演进方向
4. **[other/goal2_ToMaster.md](other/goal2_ToMaster.md)** — 架构升级思想
5. **[plan/plan_ToMaster.md](plan/plan_ToMaster.md)** — 详细实施计划
6. **[plan/chassis_dual_type_support.md](plan/chassis_dual_type_support.md)** — 底盘构型支持

### 排查历史问题
7. **[DesignComparison/deepseek_t1.md](DesignComparison/deepseek_t1.md)** — 第一轮审查
8. **[DesignComparison_2/D2result1.md](DesignComparison_2/D2result1.md)** — 第二轮修复结果（含 HAL 踩坑）
9. **[DesignComparison/A1_dp_t1.md](DesignComparison/A1_dp_t1.md)** — cmd 注释误读澄清
