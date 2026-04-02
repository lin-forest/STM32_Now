# 项目背景

STM32F103C8T6 电机末端控制器，FreeRTOS + CMSIS-OS v2，CMake 构建。
功能：CAN 接收控制命令、UART 串口调试、编码器测速、PID 控制直流电机。

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
1. **mutex 合并**：[tb6612_DC_task.c](App/tasks/tb6612_DC_task.c) 每循环 3 次 mutex acquire → 合并为一次临界区
2. **双重 osDelay 修复**：同文件末尾 `osDelay(5)` + `osDelay(10)` 叠加 → 无消息时实际延迟 15ms，应合并
3. **编码器溢出简化**：[encoder_task.c](App/tasks/encoder_task.c) 手动 if/else 溢出处理 → `diff = (int16_t)diff;`

### 功能扩展
4. **速度指令范围校验**：[command.c](App/services/command.c) `atoi()` 无边界检查 → 在 `Command_ParseString` 加 clamp
5. **死区补偿**：[speed_map.c](App/modules/speed_map.c) 低速 PWM 不足以克服静摩擦 → `ticks_to_pwm` 加最小有效 PWM 偏置
6. **PID 运行时调参（CAN 扩展）**：新增 `0x223 | data[0]=0x10/0x11/0x12` 分别设 Kp/Ki/Kd，写入 `motor_pid`，免重烧

### 按需考虑
7. **filter.c 实现**：[filter.c](App/modules/filter.c) 当前为空 → 一阶 IIR 滤波编码器噪声
8. **Logger 丢帧**：[logger_task.c](App/tasks/logger_task.c) DMA 忙时跳过 → 降低触发频率或乒乓缓冲