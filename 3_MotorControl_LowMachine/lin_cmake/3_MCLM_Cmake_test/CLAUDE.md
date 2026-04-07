# 项目背景

STM32F103C8T6 电机末端控制器，FreeRTOS + CMSIS-OS v2，CMake 构建。
功能：CAN 接收控制命令、UART 串口调试、编码器测速、PID 控制直流电机。

# 系统约束（最高优先级）

当前阶段：
- 仅实现单电机速度控制

能力边界：
- 编码器：13PPR → 低精度
- 无位置控制能力
- 无角度控制能力

禁止：
- 多电机协同
- 轨迹控制
- ROS逻辑接入

优先级：
> 控制稳定性 > 功能扩展

## 技术栈
- HAL + FreeRTOS (CMSIS-OS v2)
- 当前激活驱动：TB6612
- CAN 500kbps，UART1 DMA 日志，UART2 中断接收命令

## 目录结构
```
App/
  app_task.c / app_globals.h / app_includes.h / app_config.h
  drivers/    motor_DC_tb6612, encoder 等
  modules/    pid, speed_map
  services/   command.c/.h, logger.c/.h, can_service.c/.h
  tasks/      command_task, encoder_task, logger_task, Ack_task, heartbeat_task, *_DC_task
Core/Src/     can.c (CAN 初始化 + 中断回调), freertos.c (句柄定义)
```

## 全局关键变量
- `g_motor_status` (MotorStatus_t)：target_logic_speed, current_logic_speed, current_ticks, pwm_output
- `g_logger_enabled` (volatile uint8_t)：0=停止发送, 1=发送实时数据，默认 0
- 队列：CommandQueueHandle, MotorQueueHandle, AckQueueHandle
- 互斥：motor_mutexHandle

## CAN 协议（已确认）
| StdId  | data[0] | 含义 |
|--------|---------|------|
| 0x123/0x101/0x102 | 0x11 | CMD_SET_SPEED，value=data[1] |
| 任意   | CAN_CMD_SET_SPEED | 旧协议设速 |
| 任意   | CAN_CMD_STOP | 旧协议停止 |
| 0x223  | 0x01 | CMD_QUERY_STATUS → CAN 回复 0x323 并附带当前电机实际转速、理想转速信息 |
| 0x223  | 0x04 | CMD_LOG_START → 开始 UART1 实时数据流 |
| 0x223  | 0x05 | CMD_LOG_STOP  → 停止 UART1 实时数据流 |

CAN 回复帧 0x323，DLC=8：[target(2B), current(2B), pwm(2B), reserved(2B)]

## 任务数据流
```
CAN中断 ──────────────────────────────┐
UART2中断(行缓冲) → Command_ParseString ┤→ CommandQueue → CommandTask
                                                              ├→ MotorQueue → *_DC_Task (PID)
                                                              ├→ AckQueue  → Ack_Task (UART2回显)
                                                              └→ g_logger_enabled (直接写flag)

TIM3中断 → osThreadFlagsSet(Logger_TaHandle, 0x01)
Logger_Task: 等flag → 检查g_logger_enabled → HAL_UART_Transmit_DMA(huart1, ...)

Encoder_Task(10ms): 读TIM2计数 → 更新g_motor_status → osThreadFlagsSet(Logger_TaHandle)
```

## 编码偏好
- 最小改动原则，不加多余注释、不加防御性代码
- 不新建文件，优先改现有文件
- 不加 error handling for impossible cases


## 优化方向（已部署基线，按优先级）

### 低风险快速改动
1. **mutex 合并**：[tb6612_DC_task.c](App/tasks/tb6612_DC_task.c) ✅已完成 — 合并为一次临界区，`target_logic_speed` 无条件写在临界区顶部（stop 时也清零）
2. **双重 osDelay 修复**：同文件 ✅已完成 — 统一为 `osDelay(10)`
3. **编码器溢出简化**：[encoder_task.c](App/tasks/encoder_task.c) 手动 if/else 溢出处理 → `diff = (int16_t)diff;`

### 功能扩展
4. **速度指令范围校验**：[command.c](App/services/command.c) `atoi()` 无边界检查 → 在 `Command_ParseString` 加 clamp
5. **死区补偿**：[speed_map.c](App/modules/speed_map.c) 低速 PWM 不足以克服静摩擦 → `ticks_to_pwm` 加最小有效 PWM 偏置
6. **PID 运行时调参（CAN 扩展）**：新增 `0x223 | data[0]=0x10/0x11/0x12` 分别设 Kp/Ki/Kd，写入 `motor_pid`，免重烧

### 按需考虑
7. **filter.c 实现**：[filter.c](App/modules/filter.c) 当前为空 → 一阶 IIR 滤波编码器噪声
8. **Logger 丢帧**：[logger_task.c](App/tasks/logger_task.c) DMA 忙时跳过 → 降低触发频率或乒乓缓冲

## 编码器架构问题（待处理）

### 已确认问题
- **`SPEED_TICKS_MAX=80` 无物理依据**：应由实际最高转速反推。公式：`(RPM_max/60) * 13 * 4 * 0.01`。若电机最高 200RPM，正确值约为 17，当前值 80 严重浪费精度
- 当前电机ppr=13,stm32硬件tim-encoder模式读取*4
- 厂家表明空载转速300rpm,额定260rpm.实际负载不清楚
- **速度分辨率粗糙**：1 tick = 1.25 logic speed，低速时 PID 在离散值间跳动，控制不稳
- **`logic_to_ticks` 悬空声明**：[speed_map.h:8](App/modules/speed_map.h#L8) 有声明，[speed_map.c](App/modules/speed_map.c) 无实现 → 删除声明或补实现
- **编码器无滤波直接进 PID**：[filter.c](App/modules/filter.c) 为空，量化噪声在低速时占比高，微分项抖动
- **溢出处理冗余**：[encoder_task.c:16-18](App/tasks/encoder_task.c#L16-L18) 手动 if/else → `int16_t diff = (int16_t)(now - last_cnt);`

### 改进优先级
1. ~~实测电机最高转速，修正 `SPEED_TICKS_MAX`~~ ✅已完成 — 额定260rpm，`SPEED_TICKS_MAX=23`（原80无物理依据）；**用户后来手动回退为80**，见下方调试记录
2. ~~删除 `logic_to_ticks` 悬空声明~~ ✅已完成
3. ~~编码器溢出简化~~ ✅已完成 — `int16_t diff = (int16_t)(now - last_cnt)`
4. filter.c 一阶 IIR — 已实现但存在严重抖动问题，见下方调试记录，**当前 encoder_task 已注释掉 iir_filter 调用**
5. 多周期累积测速（M法）：累积 3~5 周期平均，提升低速精度，代价是响应延迟

---

## 抖动调试记录（进行中）

### 现象
目标速度 85，实际速度曲线高频剧烈抖动，无法跟随目标值（见截图）。

### 已改动清单

| 文件 | 改动 | 状态 |
|------|------|------|
| [app_config.h](App/app_config.h) `SPEED_TICKS_MAX` | 80 → 23（按额定260rpm计算） | ⚠️ 用户手动回退为 80 |
| [app_config.h](App/app_config.h) `ENCODER_FILTER_ALPHA` | 新增，值 0.1f | ✅ 保留 |
| [filter.h](App/modules/filter.h) | 新增 `iir_filter(int16_t, float alpha)` 声明 | ✅ 保留 |
| [filter.c](App/modules/filter.c) | 实现一阶 IIR，alpha 由调用方传入 | ✅ 保留 |
| [encoder_task.c](App/tasks/encoder_task.c) | `ticks_to_logic(iir_filter(diff, ENCODER_FILTER_ALPHA))` | ⚠️ 用户注释掉，当前未生效 |

### 根因分析

**量化噪声放大链路：**
```
1 tick 噪声
  → SPEED_TICKS_MAX=80 时：1 tick = 100/80 = 1.25 logic speed（原始）
  → SPEED_TICKS_MAX=23 时：1 tick = 100/23 = 4.35 logic speed（改后，噪声放大3.5倍）
  → Ki=17.66，每周期积分贡献：4.35 × 0.01 × 17.66 ≈ 0.77（每10ms）
  → 积分项被噪声快速推满 → 输出饱和 → 抖动
```

**IIR α=0.3 不足以抑制：**
- α=0.3 时，单次 ±1 tick 噪声经滤波后仍有 0.3 tick 透传
- 改为 α=0.1 后透传降至 0.1 tick，但 SPEED_TICKS_MAX=23 下仍 = 0.43 logic speed

**用户回退 SPEED_TICKS_MAX=80 的影响：**
- 1 tick = 1.25 logic speed，噪声幅度回到原始水平
- 但 80 远超实际最大 ticks（≈23），导致速度映射只用了量程的 29%，精度浪费

### 待验证方向

1. **PID 参数重整定** — **正在进行**。SPEED_TICKS_MAX 变化后，Ki=17.66 可能过大，需降低 Ki。
2. **重新启用 iir_filter（α=0.1）+ SPEED_TICKS_MAX=80** — **失败**。引入的相位延迟可能加剧了振荡。
3. **SPEED_TICKS_MAX 回退到 80 后，抖动是否改善？** — 已验证有改善，说明主因是量化噪声放大。
4. **多周期累积测速（M法）** — 作为根本解决方案，待后续尝试。