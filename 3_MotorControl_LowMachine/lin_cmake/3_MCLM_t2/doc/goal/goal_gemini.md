# 双电机控制系统迁移与扩展计划 (TB6612 + TIM1/TIM3)

## 1. 硬件资源重分配方案

我们将现有的单电机系统扩展为双电机系统，并将 PWM 输出全部迁移至高级定时器 `TIM1`。

| 功能组件 | 电机 1 (M1) | 电机 2 (M2) | 备注 |
| :--- | :--- | :--- | :--- |
| **PWM 定时器** | **TIM1_CH1** (PA8) | **TIM1_CH2** (PA9) | 替换原 TIM3 PWM |
| **编码器定时器**| TIM2 (PA0, PA1) | **TIM3** (PA6, PA7) | TIM3 由 PWM 转为编码器模式 |
| **方向控制 (IN1)**| **PB0** | **PB12** | GPIO_Output |
| **方向控制 (IN2)**| **PB1** | **PB13** | GPIO_Output |
| **使能引脚 (STBY)**| **PB10** | **PB14** | GPIO_Output |

## 2. 实施步骤

### 第一阶段：STM32CubeMX 硬件配置更新

1.  **配置 TIM1 (PWM 发生器)**:
    *   启用 `TIM1`。
    *   设置 `Channel 1` 和 `Channel 2` 为 `PWM Generation CHx`。
    *   参数设置：`PSC = 71`, `ARR = 999` (产生 1kHz PWM)。
    *   **注意**：TIM1 是高级定时器，其 PWM 输出需要主输出使能 (MOE)。在 HAL 库中，调用 `HAL_TIM_PWM_Start()` 即可，但需确保硬件连接正确。

2.  **配置 TIM3 (M2 编码器)**:
    *   禁用 TIM3 原有的 PWM 输出功能。
    *   将 TIM3 模式修改为 `Combined Channels -> Encoder Interface`。
    *   设置 `Encoder Mode` 为 `TI1 and TI2` (4倍频采样)。
    *   对应的 PA6, PA7 引脚将自动配置为输入。

3.  **配置 GPIO (控制引脚)**:
    *   将 **PB0, PB1, PB10** 配置为 `GPIO_Output` (M1 控制)。
    *   将 **PB12, PB13, PB14** 配置为 `GPIO_Output` (M2 控制)。
    *   初始化电平设为 `Low`。

### 第二阶段：应用层配置更新 (`App/config`)

1.  **修改 `app_config.h`**:
    ```c
    #define MOTOR_COUNT 2

    // M1 定义 (迁移至 TIM1 CH1)
    #define MOTOR1_PWM_TIM          &htim1
    #define MOTOR1_PWM_CH           TIM_CHANNEL_1
    #define MOTOR1_IN1_PIN          GPIO_PIN_0
    #define MOTOR1_IN1_PORT         GPIOB
    #define MOTOR1_IN2_PIN          GPIO_PIN_1
    #define MOTOR1_IN2_PORT         GPIOB
    #define MOTOR1_STBY_PIN         GPIO_PIN_10
    #define MOTOR1_STBY_PORT        GPIOB

    // M2 定义 (使用 TIM1 CH2 和 TIM3 编码器)
    #define MOTOR2_PWM_TIM          &htim1
    #define MOTOR2_PWM_CH           TIM_CHANNEL_2
    #define MOTOR2_IN1_PIN          GPIO_PIN_12
    #define MOTOR2_IN1_PORT         GPIOB
    #define MOTOR2_IN2_PIN          GPIO_PIN_13
    #define MOTOR2_IN2_PORT         GPIOB
    #define MOTOR2_STBY_PIN         GPIO_PIN_14
    #define MOTOR2_STBY_PORT        GPIOB
    ```

### 第三阶段：核心逻辑适配

1.  **`app_globals.c` 实例化**:
    *   更新 `g_motors` 数组，初始化两个电机的硬件句柄、PID 结构体和 ID。
    *   确保 M1 绑定 `htim2`，M2 绑定 `htim3`。

2.  **`encoder_task.c` 通用化**:
    *   修改任务循环，遍历 `g_motors` 数组。
    *   使用 `__HAL_TIM_GET_COUNTER(motor->htim_encoder)` 读取各电机对应的定时器计数值。

3.  **`motor_task.c` 实例化运行**:
    *   在 FreeRTOS 初始化时，为 `MotorControl_Task` 创建两个实例，或者在一个任务中循环处理 `MOTOR_COUNT` 个电机。

### 第四阶段：通信协议扩展

1.  **CAN 协议**:
    *   利用 CAN ID 或数据位区分电机。
    *   例如：`0x125` 数据包的 `data[7]` 位用于标识 `motor_id`。

## 3. 验证清单

- [ ] **PWM 频率验证**: 使用示波器测量 PA8, PA9，确保频率为 1kHz 且占空比可控。
- [ ] **方向 IO 验证**: 发送正反转指令，检查 PB0/1 和 PB12/13 的电平翻转。
- [ ] **编码器验证**: 手动转动电机 1 和电机 2，观察串口打印的 `current_ticks` 是否分别随之变化。
- [ ] **闭环验证**: 设定目标速度，检查两台电机是否都能稳定达到设定值。

---
**计划制定人**: Gemini Code Assist
**状态**: 准备执行 (Ready to Implement)
**日期**: 2026-04-16
```

<!--
[PROMPT_SUGGESTION]基于 goal_gemini.md 修改 app_config.h 以支持两个电机的引脚定义[/PROMPT_SUGGESTION]
[PROMPT_SUGGESTION]修改 app_globals.c 初始化 g_motors 数组以支持 TIM1 PWM 和 TIM2/3 编码器[/PROMPT_SUGGESTION]
