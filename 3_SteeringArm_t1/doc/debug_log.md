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

---

## 5. HAL_CAN_ErrorCallback — 重写 HAL 弱回调做 CAN 错误诊断

### 背景
第一次见到这种写法：在 `stm32f1xx_it.c` 里直接定义一个名为 `HAL_CAN_ErrorCallback` 的函数，HAL 就会自动调用它。

### 原理
STM32 HAL 驱动用 `__weak` 定义了一批 **空回调函数**，比如：

```c
__weak void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) { /* 空 */ }
```

用户在自己的 `.c` 文件里定义一个**同名同签名**的函数，链接器会用强符号替换弱符号，HAL 的中断服务程序会自动调用到你的版本。

不只 CAN，所有外设都有类似机制：
- `HAL_UART_ErrorCallback` / `HAL_UART_RxCpltCallback`
- `HAL_SPI_TxCpltCallback` / `HAL_SPI_RxCpltCallback`
- `HAL_TIM_PeriodElapsedCallback`
- `HAL_ADC_ConvCpltCallback`

### 当前代码做了什么

```c
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    printf("CAN_ERR: 0x%08lX\r\n", err);
    if (err & HAL_CAN_ERROR_ACK)   printf("  - ACK error (no response on bus)\r\n");
    if (err & HAL_CAN_ERROR_BOF)   printf("  - Bus-Off\r\n");
    if (err & HAL_CAN_ERROR_EPV)   printf("  - Error Passive\r\n");
    if (err & HAL_CAN_ERROR_EWG)   printf("  - Warning\r\n");
    if (err & HAL_CAN_ERROR_STF)   printf("  - Stuff error\r\n");
    if (err & HAL_CAN_ERROR_FOR)   printf("  - Form error\r\n");
    if (err & HAL_CAN_ERROR_CRC)   printf("  - CRC error\r\n");
}
```

逐位解析错误码，打印可读的故障原因——调试 CAN 通信问题时直接看到是 ACK 没应答还是总线干扰。

### 这样做的好处

| 优势 | 说明 |
|:----|:----|
| **不修改 HAL 源码** | 升级 HAL 库时不受影响 |
| **中断中快速诊断** | CAN 出错立刻知道原因，而不是静默卡死 |
| **Cortex-M 中断模型支持** | 函数名即中断向量表入口（对 IRQ Handler 而言），而回调是 HAL 内联调用，不依赖向量表 |
| **零侵入** | CubeMX 重新生成代码也不会删除你的回调函数（只要写在 `USER CODE` 段外或保留区内） |

### 注意事项 ⚠️

**`printf` 在中断中不安全**。如果 `printf` 底层走 UART 且 UART 使用中断，优先级不同可能导致死锁。推荐改成：

```c
// 只存标志，延迟处理
static volatile uint32_t g_can_err = 0;
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    g_can_err = HAL_CAN_GetError(hcan);
}
// 在主循环或 FreeRTOS 任务中检查并打印
```

### 参考
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c` 搜索 `__weak`
- RM0008 §23 — CAN 错误状态机

---

## 6. 待解决

| 问题 | 方案 | 依赖 |
|:----|:----|:----:|
| 首次激活弹向 0° | 上电读 MT6701 → 初始化 j1_current | MT6701 接线 |
| 舵机误差 | 闭环控制（MT6701 反馈 + PID） | MT6701 接线 |
| 无 CAN 状态上报 | 实现 Arm_State_Task | — |
