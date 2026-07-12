# HAL_CAN_ErrorCallback — 重写 HAL 弱回调做 CAN 错误诊断

## 这个写法是什么？

第一次见到在 `stm32f1xx_it.c` 里直接写 `HAL_CAN_ErrorCallback` 就生效了？这不是魔法，是 C 语言的**弱符号（`__weak`）**机制。

## 原理

STM32 HAL 驱动用 `__weak` 定义了一批空回调函数，比如：

```c
__weak void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) { /* 空 */ }
```

你在自己的 `.c` 文件里定义一个**同名同签名**的函数，链接器会用强符号替换弱符号，HAL 的中断服务程序会自动调用到你的版本。

## 不只是 CAN，所有外设都有

| 回调 | 外设 | 触发时机 |
|:----|:----|:----|
| `HAL_UART_ErrorCallback` | UART | 通信错误 |
| `HAL_UART_RxCpltCallback` | UART | 收到一帧数据 |
| `HAL_SPI_TxCpltCallback` | SPI | 发送完成 |
| `HAL_SPI_RxCpltCallback` | SPI | 接收完成 |
| `HAL_TIM_PeriodElapsedCallback` | TIM | 定时器溢出 |
| `HAL_ADC_ConvCpltCallback` | ADC | 转换完成 |
| `HAL_CAN_RxFifo0MsgPendingCallback` | CAN | 收到报文 |

## 当前代码

```c
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    printf("CAN_ERR: 0x%08lX\r\n", err);
    if (err & HAL_CAN_ERROR_ACK)        printf("  - ACK error (no response on bus)\r\n");
    if (err & HAL_CAN_ERROR_BOF)        printf("  - Bus-Off\r\n");
    if (err & HAL_CAN_ERROR_EPV)        printf("  - Error Passive\r\n");
    if (err & HAL_CAN_ERROR_EWG)        printf("  - Warning\r\n");
    if (err & HAL_CAN_ERROR_STF)        printf("  - Stuff error\r\n");
    if (err & HAL_CAN_ERROR_FOR)        printf("  - Form error\r\n");
    if (err & HAL_CAN_ERROR_CRC)        printf("  - CRC error\r\n");
}
```

逐位解析错误码，直接看出是 ACK 没应答、Bus-Off、还是总线干扰。

## 好处

| 优势 | 说明 |
|:----|:----|
| **不修改 HAL 源码** | 升级 HAL 库时不受影响 |
| **中断中快速诊断** | CAN 出错立刻知道原因，而不是静默卡死 |
| **零侵入** | CubeMX 重新生成代码也不会删掉你的回调 |

## ⚠️ 注意事项

**`printf` 在中断中不安全**。如果 `printf` 底层走 UART 且 UART 也使用中断，优先级不同可能导致**死锁**。

### 推荐做法：中断只存，延迟打印

```c
// 中断中 —— 只存标志
static volatile uint32_t g_can_err = 0;

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    g_can_err = HAL_CAN_GetError(hcan);
}
```

```c
// 主循环 / FreeRTOS 任务中 —— 统一打印
void CAN_Error_Print_Task(void *arg)
{
    while (1) {
        if (g_can_err) {
            uint32_t err = g_can_err;
            g_can_err = 0;

            printf("CAN_ERR: 0x%08lX\r\n", err);
            if (err & HAL_CAN_ERROR_ACK) printf("  - ACK error\r\n");
            if (err & HAL_CAN_ERROR_BOF) printf("  - Bus-Off\r\n");
            // ...
        }
        osDelay(100);
    }
}
```

## 文件位置

- `Core/Src/stm32f1xx_it.c` — 当前实现在这里（`/* USER CODE BEGIN 1 */` 段）
- 也可以放到独立文件 `App/drivers/can_monitor.c`

## 参考

- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c` 搜 `__weak`
- RM0008 §23 — CAN 错误状态机
