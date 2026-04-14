# 反馈环路修复计划

## 问题清单与修复方案

### Fix 1: `ticks_to_logic` 返回类型改为 `float`
- **文件**: `App/modules/speed_map.h` / `speed_map.c`
- **问题**: 返回 `int16_t` 赋给 `float current_logic_speed`，精度丢失，PID 微分项产生阶跃噪声
- **修复**: 将 `ticks_to_logic` 返回类型改为 `float`，内部做浮点除法

### Fix 2: Stop 时 Reset PID 积分项
- **文件**: `App/tasks/tb6612_DC_task.c`
- **问题**: `CMD_STOP` 时未调用 `PID_Reset()`，重启后积分残留导致超调冲击
- **修复**: 在 `CMD_STOP` 分支加 `PID_Reset(&motor_pid)`

### Fix 3: `logger_task` 类型截断
- **文件**: `App/tasks/logger_task.c`
- **问题**: `current_ticks` 是 `int32_t`，赋给 `int16_t speed_val` 存在截断风险
- **修复**: 将 `speed_val` 改为 `int32_t`，snprintf 格式符对应修改

## 执行状态
- [x] Fix 1: speed_map 浮点化
- [x] Fix 2: Stop 时 PID Reset
- [x] Fix 3: logger_task 类型修正
