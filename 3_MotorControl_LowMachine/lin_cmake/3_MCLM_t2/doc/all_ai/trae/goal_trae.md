# 双电机驱动改造计划

## 1. 目标

将现有单电机控制系统扩展为双电机控制系统。

- **电机1**: PWM 控制由 `TIM3` 迁移至 `TIM1` 的通道1和2。编码器反馈继续使用 `TIM2`。
- **电机2**: 新增的电机，其 PWM 控制使用 `TIM1` 的通道3和4，编码器反馈使用 `TIM3`。

## 2. 硬件资源重分配

| 功能            | 原配置                | 新配置                  | 备注                               |
| :-------------- | :-------------------- | :---------------------- | :--------------------------------- |
| **电机1 PWM**   | `TIM3_CH1`, `TIM3_CH2`  | `TIM1_CH1`, `TIM1_CH2`    | `TIM1` 为高级定时器，更适合电机控制 |
| **电机1 编码器** | `TIM2` (Encoder Mode)   | `TIM2` (Encoder Mode)     | 保持不变                           |
| **电机2 PWM**   | N/A                   | `TIM1_CH3`, `TIM1_CH4`    | 使用 `TIM1` 的剩余通道              |
| **电机2 编码器** | N/A                   | `TIM3` (Encoder Mode)     | `TIM3` 从 PWM 功能转换而来         |
| **空闲定时器**   | `TIM1`                | N/A                     | `TIM1` 被充分利用                   |

## 3. 实施步骤

### 第一阶段：CubeMX 配置修改

1.  **配置 TIM1**:
    -   启用 `TIM1`。
    -   配置 `Channel 1`, `Channel 2`, `Channel 3`, `Channel 4` 为 `PWM Generation CHx` 模式。
    -   根据电机驱动芯片要求，考虑启用 `Break and Dead-Time` 功能。
    -   为 `TIM1_CH1/2/3/4` 分配具体的 GPIO 引脚。

2.  **配置 TIM3**:
    -   禁用 `TIM3` 的 PWM 输出通道。
    -   将 `TIM3` 的模式更改为 `Encoder Interface`，并选择 `Encoder Mode TI1 and TI2`。
    -   为 `TIM3_CH1` 和 `TIM3_CH2` 分配 GPIO 引脚作为编码器输入。
    -   根据需要配置编码器参数（如 `Polarity`, `Prescaler` 等）。

3.  **GPIO 引脚规划**:
    -   仔细规划并记录所有新分配的引脚，避免冲突。
    -   `TIM1_CH1/2` -> 电机1 PWM 输入。
    -   `TIM1_CH3/4` -> 电机2 PWM 输入。
    -   `TIM3_CH1/2` -> 电机2 编码器 A/B 相。

4.  **生成代码**:
    -   保存 `.ioc` 文件并重新生成代码。这将更新 `main.c`, `tim.c`, `tim.h` 等相关文件。

### 第二阶段：软件代码适配

1.  **初始化定时器 (main.c 或 tim.c)**:
    -   确认 `MX_TIM1_Init()` 和 `MX_TIM3_Init()` 被正确调用。
    -   在初始化代码中，启动 `TIM1` 的4个PWM通道和 `TIM3` 的编码器接口。
        ```c
        // 启动 TIM1 PWM
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        // 启动 TIM3 编码器
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        ```
    -   移除或注释掉原有的 `HAL_TIM_PWM_Start(&htim3, ...)` 调用。

2.  **适配电机1控制任务 (App/tasks/tb6612_DC_task.c)**:
    -   将文件中所有对 `htim3` 的引用替换为 `htim1`。
    -   修改 `__HAL_TIM_SET_COMPARE()` 宏，使其指向 `htim1` 和正确的通道 (`TIM_CHANNEL_1`, `TIM_CHANNEL_2`)。
    -   建议将此文件重命名为 `motor1_task.c` 以提高代码可读性。

3.  **创建电机2相关任务**:
    -   **`motor2_task.c`**: 复制 `motor1_task.c` 并修改，创建一个新的任务 `Motor2Task`，使其使用 `htim1` 的 `TIM_CHANNEL_3` 和 `TIM_CHANNEL_4`。为电机2创建独立的 PID 和状态变量。
    -   **`speed2_get_task.c`**: 复制 `speed_get_task.c` 并修改，创建一个新的任务 `Speed2GetTask`，使其从 `htim3` (`__HAL_TIM_GET_COUNTER(&htim3)`) 读取编码器值，并更新电机2的状态。

4.  **扩展命令和状态管理**:
    -   **`app_config.h`**: 为电机2定义新的 PID 参数和速度常量。
    -   **`command.h`**: 扩展 `CommandMsg` 结构体，增加一个 `motor_id` 字段，以便命令可以区分目标电机。
        ```c
        typedef struct {
            CommandType type;
            uint8_t motor_id; // 1 for motor1, 2 for motor2
            int32_t value;
        } CommandMsg;
        ```
    -   **`can.c` / `usart.c`**: 修改命令解析逻辑，以支持对不同电机的控制。
    -   **`logger_task.c`**: 扩展日志功能，使其可以同时记录两个电机的状态。

### 第三阶段：集成与测试

1.  **编译与调试**:
    -   逐个文件修改后，解决编译错误。
    -   首先专注于让电机1使用 `TIM1` 正常工作。
    -   然后，逐步集成电机2的控制和反馈代码。
2.  **单元测试**:
    -   单独测试电机2的 PWM 输出是否正确。
    -   单独测试电机2的编码器读数是否准确。
3.  **集成测试**:
    -   同时向两个电机发送指令，观察它们是否能独立、正确地响应。
    -   检查日志输出，确认两个电机的数据都被正确记录。
    -   测试系统在双电机运行时的负载和实时性。

