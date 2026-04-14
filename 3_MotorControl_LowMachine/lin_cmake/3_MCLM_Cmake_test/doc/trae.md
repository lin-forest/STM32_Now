
# 项目代码审查与数据流分析

本文档旨在全面审查 `3_MCLM_Cmake_test` 项目的代码结构、接口设计和核心数据流。

## 架构
lin@lin-virtual-machine:~/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/3_MotorControl_LowMachine/lin_cmake/3_MCLM_Cmake_test$ tree -L 3
.
├── 3_MCLM_Cmake_test.ioc
├── App
│   ├── config
│   │   ├── app_config.h
│   │   ├── app_globals.h
│   │   ├── app_includes.h
│   │   ├── app_task.c
│   │   └── app_task.h
│   ├── drivers
│   │   ├── motor_DC_at8236.c
│   │   ├── motor_DC_at8236.h
│   │   ├── motor_DC_tb6612.c
│   │   └── motor_DC_tb6612.h
│   ├── modules
│   │   ├── filter.c
│   │   ├── filter.h
│   │   ├── pid.c
│   │   ├── pid.h
│   │   ├── speed_map.c
│   │   └── speed_map.h
│   ├── services
│   │   ├── command.c
│   │   ├── command.h
│   │   ├── logger.c
│   │   └── logger.h
│   └── tasks
│       ├── Ack_task.c
│       ├── at8236_DC_task.c
│       ├── command_task.c
│       ├── encoder_task.c
│       ├── heartbeat_task.c
│       ├── logger_task.c
│       └── tb6612_DC_task.c
├── build
│   └── Debug
│       ├── 3_MCLM_Cmake_test.elf
│       ├── 3_MCLM_Cmake_test.map
│       ├── build.ninja
│       ├── cmake
│       ├── CMakeCache.txt
│       ├── CMakeFiles
│       ├── cmake_install.cmake
│       └── compile_commands.json
├── cmake
│   ├── gcc-arm-none-eabi.cmake
│   ├── starm-clang.cmake
│   └── stm32cubemx
│       └── CMakeLists.txt
├── CMakeLists.txt
├── CMakePresets.json
├── Core
│   ├── Inc
│   │   ├── can.h
│   │   ├── dma.h
│   │   ├── FreeRTOSConfig.h
│   │   ├── gpio.h
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   ├── stm32f1xx_it.h
│   │   ├── tim.h
│   │   └── usart.h
│   └── Src
│       ├── can.c
│       ├── dma.c
│       ├── freertos.c
│       ├── gpio.c
│       ├── main.c
│       ├── stm32f1xx_hal_msp.c
│       ├── stm32f1xx_hal_timebase_tim.c
│       ├── stm32f1xx_it.c
│       ├── syscalls.c
│       ├── sysmem.c
│       ├── system_stm32f1xx.c
│       ├── tim.c
│       └── usart.c
├── doc
│   ├── CLAUDE.md
│   ├── goal.md
│   ├── result.md
│   └── trae.md
├── Drivers
│   ├── CMSIS
│   │   ├── Device
│   │   ├── Include
│   │   └── LICENSE.txt
│   └── STM32F1xx_HAL_Driver
│       ├── Inc
│       ├── LICENSE.txt
│       └── Src
├── flash.jlink
├── Middlewares
│   └── Third_Party
│       └── FreeRTOS
├── newlib_lock_glue.c
├── startup_stm32f103xb.s
├── STM32F103XX_FLASH.ld
└── stm32_lock.h

26 directories, 71 files

## 1. 项目结构概览

项目采用分层和模块化的结构，代码组织清晰，主要逻辑位于 `App` 目录下：

-   **`App/config`**: 存放全局配置，如硬件选择、PID参数、CAN ID等。是系统配置的单一入口点。
-   **`App/services`**: 提供应用层服务，如命令解析 (`command`) 和日志记录 (`logger`)。
-   **`App/modules`**: 包含核心算法模块，如PID控制器 (`pid`) 和速度映射 (`speed_map`)。
-   **`App/drivers`**: 硬件驱动层，封装了对具体电机驱动芯片（如TB6612, AT8236）的底层操作。
-   **`App/tasks`**: RTOS任务实现，负责调度和组织各个模块，形成完整的业务逻辑。

## 2. 核心数据结构与全局变量

数据交换的核心是 `app_globals.h` 中定义的全局变量和RTOS内核对象。

### `MotorStatus_t g_motor_status`

这是最核心的全局状态结构体，用于在各个任务之间共享电机的实时状态。

-   `float target_logic_speed`: 目标逻辑速度 (范围 -100.0 ~ 100.0)，由控制指令设定。
-   `float current_logic_speed`: 当前实际的逻辑速度，由 `encoder_task` 计算。
-   `int32_t current_ticks`: 编码器在单个采样周期内的原始计数值。
-   `int16_t pwm_output`: 当前施加到电机的PWM值。

### RTOS 内核对象

-   `osMutexId_t motor_mutexHandle`: 互斥锁，用于保护对 `g_motor_status` 的并发访问，防止数据竞争。
-   `osMessageQueueId_t CommandQueueHandle`: 消息队列，用于从串口接收任务 (`logger.c`) 向命令处理任务 (`command_task`) 传递解析后的命令。
-   `osMessageQueueId_t MotorQueueHandle`: 消息队列，用于从 `command_task` 向具体的电机控制任务 (如 `tb6612_DC_task`) 传递最终的速度设定指令。

## 3. 核心数据流分析

系统的数据流可以分为“上行控制流”和“下行反馈流”，两者在电机控制任务中汇合，构成闭环控制。

### 3.1. 上行控制流 (指令下发)

此流程描述了从外部输入（如PC串口）到电机执行的完整路径。

1.  **物理输入 (UART)**:
    -   **文件**: `App/services/logger.c`
    -   **函数**: `HAL_UART_RxCpltCallback()`
    -   **流程**: PC通过串口发送字符串命令 (如 `"S100\n"`)。`HAL_UART_RxCpltCallback` 中断服务程序以行缓冲的方式接收字符串。

2.  **命令解析**:
    -   **文件**: `App/services/command.c`
    -   **函数**: `Command_ParseString()`
    -   **流程**: `HAL_UART_RxCpltCallback` 接收到完整的一行后，调用 `Command_ParseString()` 将字符串 `"S100"` 解析为 `CommandMsg_t {type: CMD_SET_SPEED, value: 100}`。

3.  **命令分发 (队列1)**:
    -   **文件**: `App/services/logger.c`
    -   **流程**: 解析后的 `CommandMsg_t` 被放入 `CommandQueueHandle` 消息队列。

4.  **命令处理与转发 (队列2)**:
    -   **文件**: `App/tasks/command_task.c` (推测)
    -   **流程**: `command_task` 从 `CommandQueueHandle` 中取出消息，进行预处理或直接转发，将最终的控制指令（同样是 `CommandMsg_t` 结构）放入 `MotorQueueHandle` 消息队列。

5.  **PID目标设定**:
    -   **文件**: `App/tasks/tb6612_DC_task.c`
    -   **函数**: `TB6612_DC_Task()`
    -   **流程**: 电机控制任务从 `MotorQueueHandle` 获取指令，并更新其内部的PID控制器目标值：`motor_pid.setpoint = (float)cmdMsg.value;`。

### 3.2. 下行反馈流 (编码器数据)

此流程描述了如何从编码器获取速度反馈。

1.  **硬件采样 (TIM)**:
    -   **文件**: `Core/Src/tim.c`
    -   **流程**: 编码器定时器 (如 `TIM2`) 工作在编码器模式，硬件自动对AB相脉冲进行计数，计数值保存在 `TIM2->CNT` 寄存器中。

2.  **速度计算**:
    -   **文件**: `App/tasks/encoder_task.c`
    -   **流程**: `encoder_task` 每隔一个固定的采样周期（例如10ms）执行一次。
        a. 读取 `TIM2->CNT` 的值，得到该周期内的编码器增量 `raw_ticks`。
        b. `g_motor_status.current_ticks = raw_ticks;`
        c. 调用 `ticks_to_logic(raw_ticks)` 将物理计数值归一化为逻辑速度。
        d. `g_motor_status.current_logic_speed = logic_speed;`

### 3.3. 闭环控制核心

上行流和下行流在电机控制任务中汇合，形成闭环。

-   **文件**: `App/tasks/tb6612_DC_task.c`
-   **周期**: 10ms (由 `osDelay(10)` 控制)
-   **流程**:
    1.  **获取互斥锁**: `osMutexAcquire(motor_mutexHandle, ...)` 确保数据同步。
    2.  **获取输入**:
        -   目标值: `motor_pid.setpoint` (来自上行控制流)
        -   反馈值: `g_motor_status.current_logic_speed` (来自下行反馈流)
    3.  **PID计算**: 调用 `PID_Compute(&motor_pid, current_speed)`，计算出控制输出 `output` (一个逻辑值)。
    4.  **硬件执行**:
        -   调用 `TB6612_Motor_SetSpeed(&tb6612_motor, (int16_t)output)`。
        -   该函数内部将逻辑值 `output` 转换为PWM占空比，并设置GPIO电平控制方向，最终驱动电机转动。
    5.  **状态更新**: 将计算出的PWM值更新回 `g_motor_status.pwm_output`，供日志任务显示。
    6.  **释放互斥锁**: `osMutexRelease(motor_mutexHandle)`。

## 4. 接口与模块说明

### `App/config`

-   `app_config.h`: 核心配置文件，通过宏定义控制硬件选择、PID参数、引脚分配等。修改此处可以适配不同硬件。
-   `app_globals.h`: 全局变量和RTOS对象声明。

### `App/services`

-   `command.h` / `command.c`:
    -   `CommandMsg_t Command_ParseString(const char *cmdStr)`: 将字符串解析为命令结构体。
-   `logger.h` / `logger.c`:
    -   `LoggerStatus_t Logger_Printf(const char *fmt, ...)`: 异步、非阻塞的格式化串口打印函数。
    -   `HAL_UART_RxCpltCallback()`: 串口接收中断，负责接收命令并投递到队列。

### `App/modules`

-   `pid.h` / `pid.c`:
    -   `void PID_Init(...)`: 初始化PID控制器参数。
    -   `float PID_Compute(PID_Controller *pid, float current_value)`: 执行一次PID计算，返回控制量。
    -   `void PID_Reset(PID_Controller *pid)`: 复位PID状态。
-   `speed_map.h` / `speed_map.c`:
    -   `int16_t ticks_to_logic(int16_t ticks)`: 将编码器ticks转换为逻辑速度。
    -   `int16_t logic_to_pwm(int16_t logic)`: 将逻辑速度转换为PWM值 (此项目中未使用，功能被驱动层函数替代)。

### `App/drivers`

-   `motor_DC_tb6612.h` / `motor_DC_tb6612.c`:
    -   `void TB6612_Motor_Init(...)`: 初始化电机驱动硬件。
    -   `void TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int16_t speed)`: 设置电机速度和方向。
    -   `void TB6612_Motor_Stop(TB6612_Motor_t *motor)`: 停止电机（刹车或滑行）。

### `App/tasks`

-   `tb6612_DC_task.c`: TB6612电机的控制任务，是闭环控制的核心调度器。
-   `encoder_task.c`: 编码器数据采集任务，负责计算 `current_logic_speed`。
-   `logger_task.c`: 日志输出任务，周期性地通过 `Logger_Printf` 打印 `g_motor_status` 的内容。
-   `command_task.c`: 命令处理任务，负责对接 `CommandQueueHandle` 和 `MotorQueueHandle`。

