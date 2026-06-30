# 机械臂控制器 — 逐步实施过程

> 配合 `plan_steering_arm.md` 使用，按顺序逐步完成。
> 每步都有明确的目标和验收标准，完成后打勾 ✅

## 整体进度

| Phase | 内容 | 进度 |
|:----:|:----|:----:|
| 1~2 | 基础验证（LED/printf/CAN） | ✅ 完成 |
| **3** | **舵机 + MT6701** | **🔶 核心完成** |
| 4 | J0 直流电机 | ⏳ 暂缓 |
| 5 | CAN 状态上报 | ⏳ 待开始 |
| 6 | 网关适配 | ⏳ 待开始 |

---

## Phase 0：准备工作

- [ ] 安装 STM32CubeMX（确认版本 ≥ 6.12）
- [ ] 确认 VS Code + STM32 扩展（`stm32cube-ide-clangd`, `cube-cmake`）可用
- [ ] 确认 JLink/ST-Link 驱动正常
- [ ] 准备硬件：STM32F103C8T6 核心板 × 1、TB6612 模块 × 1、舵机 × 4、MT6701 × 2、CAN 收发器模块（如 TJA1050）× 1

---

## Phase 1：CubeMX 生成基础工程

> 按 `plan_steering_arm.md` 第五章配置，这里给出 CubeMX 操作的**点击次序**。

### Step 1.1 — 新建工程

```
1. File → New Project → 选 STM32F103C8T6 → Start Project
2. Pinout & Configuration 视图中：
```

### Step 1.2 — RCC

```
System Core → RCC
  High Speed (HSE) → Crystal/Ceramic Resonator
```

### Step 1.3 — 时钟配置

```
Clock Configuration 标签页
  HSE (8MHz) → PLL (x9) → SYSCLK = 72MHz
  APB1 = 36MHz (/2)
  APB2 = 72MHz (/1)
```

### Step 1.4 — 逐个配置外设（按下面顺序）

#### CAN1
```
Connectivity → CAN1
  Mode → Activate
  参数 (Parameter Settings):
    Prescaler = 4
    BS1 = 13 TQ
    BS2 = 4 TQ
    SJW = 1 TQ
    Time Triggered Communication = Disable
    Auto Bus-Off = Enable
    Auto Wake-Up = Disable
    Auto Retransmission = Enable
    Receive Fifo Locked = Disable
    Transmit Fifo Priority = Enable
  NVIC Settings:
    ☑ USB low priority or CAN RX0 interrupts → 优先级 5
    ☑ CAN SCE interrupt → 优先级 5
    ☐ USB high priority or CAN TX interrupts ← 不勾
```

> **CAN NVIC 三个中断的解释：**
> 
> | CubeMX 显示名 | 向量名 | 触发时机 | 你需要的回调 | 勾？ |
> |------|--------|---------|------------|:---:|
> | `USB low priority or CAN RX0` | `USB_LP_CAN1_RX0_IRQn` | **收到 CAN 帧**（FIFO0 有数据） | `HAL_CAN_RxFifo0MsgPendingCallback()` | **✅ 必须** |
> | `CAN SCE` | `CAN1_SCE_IRQn` | **状态/错误变化**（总线 Off、错误被动、仲裁丢失） | `HAL_CAN_ErrorCallback()` | **✅ 建议** |
> | `USB high priority or CAN TX` | `USB_HP_CAN1_TX_IRQn` | **帧发送完成**（成功送到总线） | `HAL_CAN_TxMailboxNCompleteCallback()` | ❌ 不开 |
> 
> - **RX0**（必须开）：没这个你收不到任何 CAN 帧。回调里 `HAL_CAN_GetRxMessage()` 取出数据 → 投递到 FreeRTOS 队列
> - **SCE**（强烈建议开）：调试联调时极有用。总线 Off 时你可以在 `HAL_CAN_ErrorCallback()` 里打印错误码并尝试恢复：
>   ```c
>   void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
>       uint32_t err = HAL_CAN_GetError(hcan);
>       if (err & HAL_CAN_ERROR_BOFF)
>           printf("CAN BUS OFF! 检查接线/波特率\r\n");
>       if (err & HAL_CAN_ERROR_EWG)
>           printf("CAN Error Warning (err_cnt ≥ 96)\r\n");
>   }
>   ```
> - **TX**（不开）：发帧是同步操作，看 `HAL_CAN_AddTxMessage()` 返回值 `HAL_OK` 就够了。现阶段不需要发送完成通知

#### USART1（调试串口）
```
Connectivity → USART1
  Mode → Asynchronous
  Parameter Settings:
    Baud Rate = 115200
    Word Length = 8 Bits
    Parity = None
    Stop Bits = 1
```

#### SPI1（MT6701 共用）
```
Connectivity → SPI1
  Mode → Full Duplex Master
  Parameter Settings:
    Frame Format → Motorola
    Data Size → 8 Bits
    First Bit → MSB First
    Prescaler → 64 (→ 1.125MHz)
    CPOL → Low
    CPHA → 2 Edge     ← ❗ 实测必须 Mode 1
    NSS → Software
```

#### TIM2（J0 编码器）
```
Timers → TIM2
  Slave Mode → Encoder Mode TI1 and TI2
  Channel 1 (PA0) → TI1
  Channel 2 (PA1) → TI2
  Parameter Settings:
    Prescaler = 0
    Counter Period = 65535
    Auto-reload = Enable
    Slave Mode = Encoder Mode 3
```

#### TIM1（J0 直流电机 PWM）

与 `3_MCLM_t2` 保持一致，便于直接复用 `PWM_MAX=7200` 的速度映射代码。

```
Timers → TIM1
  Channel 1 (PA8) → PWM Generation CH1
  Parameter Settings:
    Prescaler = 0            ← 不分频 (72MHz)
    Counter Period = 7200-1  ← 72MHz/7200 = 10kHz (与 MCLM_t2 一致)
    Auto-reload = Enable
  PWM1 CH1:
    Pulse = 0
    Polarity = High
```

> **为什么不用 20kHz？** 见下方对比。10kHz 的分辨率更高（7200 级），且与 `MCLM_t2` 的 `PWM_MAX=7200` 完全兼容，`speed_map` 代码不用改系数。

#### TIM4（4 路舵机 PWM）— 35kg 300° 数字舵机

舵机参数：**500μs~2500μs 对应 0°~300°**

```
Timers → TIM4
  Channel 1 (PB6) → PWM Generation CH1  ← J1 舵机
  Channel 2 (PB7) → PWM Generation CH2  ← J2 舵机
  Channel 3 (PB8) → PWM Generation CH3  ← 夹爪1
  Channel 4 (PB9) → PWM Generation CH4  ← 夹爪2
  Parameter Settings:
    Prescaler = 18-1        ← 36MHz/18 = 2MHz → 每 tick = 0.5μs
    Counter Period = 40000-1  ← 2MHz/40000 = 50Hz ✅ (40000 < 65535)
    Auto-reload = Enable
  各通道：
    Pulse = 3000            ← 初始 150°（中间位置）
    Polarity = High
```

> **为什么旧参数不合适：** 旧配置 PSC=720 每 tick=20μs，500~2500μs 只有 100 个档位，300°/100=3°/步，太粗。
> 新配置每 tick=0.5μs，可调范围 4000 档，300°/4000=**0.075°/步**，精度提升 40 倍。
>
> 脉宽映射关系：
> ```
> 500μs   = 0.5μs × 1000  → CCR = 1000 → 0°
> 1500μs  = 0.5μs × 3000  → CCR = 3000 → 150°（中间）
> 2500μs  = 0.5μs × 5000  → CCR = 5000 → 300°
> ```

#### GPIO
```
点击 PB1 → 设为 Output Push-Pull, label = "J1_CS", 初始 High
点击 PB12 → 设为 Output Push-Pull, label = "J2_CS", 初始 High
点击 PA2 → 设为 Output Push-Pull, label = "J0_IN1", 初始 Low
点击 PA3 → 设为 Output Push-Pull, label = "J0_IN2", 初始 Low
PC13 默认即是 Output，label = "LED"
```

#### ⚠️ HAL 时间基准 —— 容易忘！
```
Project Manager → Project Settings
  → Pinout → HAL Settings → Timebase Source
  → 从 TIM1 改为 TIM3
```

#### FreeRTOS
```
Middleware → FREERTOS
  Interface → CMSIS_V2
  Config Parameters → Kernel Settings:
    USE_NEWLIB_REENTRANT = Enable
  Config Parameters → Memory Management:
    Heap Size = 16384
```

### Step 1.5 — 生成代码

```
Project Manager → Project
  Project Name = 3_SteeringArm
  Project Location = 指向 3_SteeringArm/
  Toolchain / IDE = CMake (或 STM32CubeIDE)
  Generate Code → Open Project
```

**验收标准**：✅ CubeMX 生成成功，项目结构完整，无红色错误。

---

## Phase 2：基础验证（CubeMX 生成后）

### Step 2.1 — 首次编译 & 烧录

```bash
cd 3_SteeringArm
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build
# 烧录
JLinkExe -Device STM32F103C8 -If SWD -Speed 4000 -CommanderScript flash.jlink
```

### Step 2.2 — 验证 HAL 时间基准 + FreeRTOS 启动

```c
// 在 Core/Src/main.c 的 USER CODE BEGIN 2 中添加：

/* USER CODE BEGIN 2 */
printf("3_SteeringArm starting...\r\n");
/* USER CODE END 2 */
```

然后让 `Heartbeat_Task` 闪烁 PC13 LED（在 `freertos.c` 的任务入口中添加）：

```c
// freertos.c 中找到 MX_FREERTOS_Init() 附近，在 USER CODE 区域添加：
/* Create the Heartbeat task */
osThreadAttr_t heartbeat_attr = {
    .name = "Heartbeat_Ta",
    .priority = osPriorityLow,
    .stack_size = 64,
};
osThreadNew(Heartbeat_Task, NULL, &heartbeat_attr);
```

```c
// 新建 App/tasks/heartbeat_task.c
#include "app_includes.h"

void Heartbeat_Task(void *argument)
{
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(300);
    }
}
```

**验收标准**：✅ 板载 LED 以 300ms 间隔闪烁。串口（115200）能看到 "3_SteeringArm starting..."

### Step 2.3 — 验证 USART1 打印浮点数（可选）

```c
printf("Test float: %.2f\r\n", 3.14159f);
```

如果只显示 `Test float: ` 没有数字 → 需要加 `_printf_float`：

```cmake
# cmake/gcc-arm-none-eabi.cmake 中添加
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,_printf_float")
```

代价：Flash 增加 ~12KB。如果空间紧张，把浮点改成整数打印。
**验收标准**：✅ 串口能看到完整数字。

### Step 2.4 — 验证 CAN 通信

连接 CAN 收发器 + USB 转 CAN（如 CANable），主机开 SavvyCAN。

添加 CAN RX 任务接收 CAN 帧：

```c
// freertos.c 中添加：
osMessageQueueId_t canRxQueueHandle;

// 在 MX_FREERTOS_Init() 中：
canRxQueueHandle = osMessageQueueNew(16, sizeof(App_CAN_Message_t), NULL);

// 创建 CAN_Rx 任务
osThreadAttr_t can_rx_attr = {
    .name = "CAN_Rx_Ta",
    .priority = osPriorityNormal,
    .stack_size = 256,
};
osThreadNew(CAN_Rx_Task, NULL, &can_rx_attr);
```

```c
// App/tasks/can_rx_task.c
#include "app_includes.h"
#include "command.h"

osMessageQueueId_t canRxQueueHandle;

void CAN_Rx_Task(void *argument)
{
    App_CAN_Message_t msg;
    
    for (;;) {
        if (osMessageQueueGet(canRxQueueHandle, &msg, NULL, osWaitForever) == osOK) {
            printf("CAN RX: ID=0x%03X Len=%d ", msg.id, msg.len);
            for (int i = 0; i < msg.len; i++)
                printf("%02X ", msg.data[i]);
            printf("\r\n");
        }
    }
}
```

```c
// App/services/command.h
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} App_CAN_Message_t;
```

```c
// stm32f1xx_it.c 中，HAL_CAN_RxFifo0MsgPendingCallback 回调：
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    App_CAN_Message_t msg;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data);
    msg.id = msg.header.StdId;
    msg.len = msg.header.DLC;
    osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
}
```

在 SavvyCAN 上发一帧 `0x130 11 00 00 00 00 00 00 00`，看串口是否有打印。

**验收标准**：✅ 串口打印 `CAN RX: ID=0x130 Len=8 11 00 00 00 00 00 00 00`

### Step 2.5 — CAN 定时发送（验证 TX）

```c
// 在 CAN_Rx_Task 中加入：收到 0x230 (ARM_QUERY) 时回复一帧
if (msg.id == 0x230) {
    App_CAN_Message_t tx_msg = {
        .id = 0x330,
        .len = 8,
        .data = {0, 0, 0, 0, 0, 0, 0, 0}
    };
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = tx_msg.id,
        .DLC = tx_msg.len,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
    };
    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_msg.data, &tx_mailbox);
}
```

SavvyCAN 发送 `0x230 00`，看是否能收到 `0x330` 的回复。

**验收标准**：✅ CAN 总线能正常收发

---

## Phase 4：J0 直流电机驱动（暂缓）

### Step 4.1 — 移植 TB6612 驱动代码

从 `3_MCLM_t2` 复制以下文件到 `3_SteeringArm/App/`：

```bash
# 从 3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/App/ 复制
cp -r modules/pid.*  目标:App/modules/
cp -r modules/filter.* 目标:App/modules/
cp -r modules/speed_map.* 目标:App/modules/
cp -r drivers/motor_DC_tb6612.* 目标:App/drivers/
```

### Step 4.2 — 配置 J0 电机参数

新建 `App/config/app_arm_config.h`：

```c
#ifndef __APP_ARM_CONFIG_H__
#define __APP_ARM_CONFIG_H__

/* J0 直流电机参数 (TB6612) */
#define J0_PWM_TIM        &htim1
#define J0_PWM_CHANNEL    TIM_CHANNEL_1
#define J0_IN1_PORT       GPIOA
#define J0_IN1_PIN        GPIO_PIN_2
#define J0_IN2_PORT       GPIOA
#define J0_IN2_PIN        GPIO_PIN_3

/* J0 编码器参数 */
#define J0_ENCODER_TIM    &htim2
#define J0_ENCODER_PPR    11          // 根据实际编码器修改
#define J0_ENCODER_TI12   4           // 4倍频

/* PID 参数 (初值，后续整定) */
#define J0_PID_KP         0.4584f
#define J0_PID_KI         17.66f
#define J0_PID_KD         0.0025f
#define J0_PID_TS         0.01f       // 10ms
#define J0_PID_OUTPUT_LIMIT  100.0f

/* 速度映射 */
#define J0_SPEED_TICKS_MAX  96        // 参考3_MCLM参数
#define J0_SPEED_LOGIC_MAX  100
#define J0_PWM_MAX          3600      // TIM1_CH1 的 ARR+1

#endif
```

### Step 4.3 — 实现 J0 电机控制任务

```c
// App/tasks/dc_motor_task.c
#include "app_includes.h"
#include "pid.h"
#include "motor_DC_tb6612.h"
#include "speed_map.h"
#include "app_arm_config.h"

extern osMessageQueueId_t jointCmdQueueHandle;
extern ArmState_t g_arm_state;

static TB6612_Motor_t g_j0_motor;
static PID_Controller g_j0_pid;

void DC_Motor_Task(void *argument)
{
    // 初始化 TB6612 驱动
    TB6612_Init(&g_j0_motor,
                J0_PWM_TIM, J0_PWM_CHANNEL,
                J0_IN1_PORT, J0_IN1_PIN,
                J0_IN2_PORT, J0_IN2_PIN);
    TB6612_Stop(&g_j0_motor, TB6612_MOTOR_STOP_BRAKE);

    // 初始化 PID
    PID_Init(&g_j0_pid,
             J0_PID_KP, J0_PID_KI, J0_PID_KD,
             5.66f, J0_PID_OUTPUT_LIMIT,
             J0_PID_TS, 0.3f);

    // 清零编码器
    __HAL_TIM_SET_COUNTER(J0_ENCODER_TIM, 0);
    HAL_TIM_Encoder_Start(J0_ENCODER_TIM, TIM_CHANNEL_ALL);

    Arm_JointCmd_t cmd;

    for (;;) {
        // 1. 检查是否有新命令
        int16_t target = g_arm_state.j0_target;  // 默认用全局目标值
        if (osMessageQueueGet(jointCmdQueueHandle, &cmd, NULL, 0) == osOK) {
            if (cmd.joint_id == 0) {
                g_arm_state.j0_target = cmd.value;
                target = cmd.value;
            }
        }

        // 2. 读取编码器
        int32_t ticks = (int16_t)__HAL_TIM_GET_COUNTER(J0_ENCODER_TIM);
        __HAL_TIM_SET_COUNTER(J0_ENCODER_TIM, 0);  // 清零，下次读差值
        g_arm_state.j0_ticks = ticks;

        float current_speed = ticks_to_logic(ticks, J0_SPEED_TICKS_MAX);

        // 3. PID 计算
        float output = PID_Update(&g_j0_pid, target, current_speed);

        // 4. 输出 PWM
        int16_t pwm = logic_to_pwm(output, J0_PWM_MAX, J0_SPEED_LOGIC_MAX);
        TB6612_SetSpeed(&g_j0_motor, pwm);

        // 5. 更新全局状态
        g_arm_state.j0_current_speed = (int16_t)current_speed;
        g_arm_state.j0_pwm = pwm;

        osDelay(10);  // 10ms = PID 周期
    }
}
```

> 注：`ticks_to_logic()` 和 `logic_to_pwm()` 需要根据 `speed_map.c` 适配参数。

### Step 4.4 — 测试 J0 空载

1. 烧录程序
2. 不接电机（仅通电），观察 printf 输出
3. 通过 CAN 发送命令 `0x130 | 11 00 32 00 00 00 00 00`（J0 目标速度 = 50）
4. 接电机后，观察 PWM 输出和编码器反馈

**验收标准**：✅ J0 电机可按指定速度转动，PID 稳定无振荡

---

## Phase 3：舵机 + MT6701 编码器 — 🔶 核心完成

| 子步骤 | 内容 | 状态 |
|:----:|------|:----:|
| 3.1 | MT6701 SPI 驱动 | ✅ 代码就绪，待接线测试 |
| 3.2 | 舵机 PWM 封装 (500~2500μs, 300°) | ✅ 已验证300°全行程 |
| 3.3 | Servo_Task 平滑插值 + CAN控制 | ✅ speed_dps可调 |
| 3.4 | 单步测试 | ✅ 上电无动作，CAN激活 |

> **注意**：当前 Servo_Task 已从扫描测试模式改为生产模式：
> - 上电 `g_servo_active=0` → 不输出 PWM，舵机保持原位
> - 收到 CAN 0x130 后 `g_servo_active=1` → 开始平滑插值输出
> - 默认速度 180°/s，可用 0x430 调整

### Step 3.1 — 移植 MT6701 驱动 ✅ 代码已创建

参考 `6_MT6701/6_mt6701_spi/Core/Src/main.c` 中的读取函数，封装为独立驱动：

```c
// App/drivers/mt6701.h
#ifndef __MT6701_H__
#define __MT6701_H__

#include <stdint.h>

void    MT6701_Init(void);
uint16_t MT6701_ReadRaw(GPIO_TypeDef *cs_port, uint16_t cs_pin);
float   MT6701_GetAngle(GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif
```

```c
// App/drivers/mt6701.c
#include "mt6701.h"
#include "spi.h"

#define MT6701_CS_LOW(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define MT6701_CS_HIGH(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

void MT6701_Init(void)
{
    // SPI 已在 CubeMX 初始化，只需确保 CS 引脚初始为 High
    MT6701_CS_HIGH(J1_CS_GPIO_Port, J1_CS_Pin);
    MT6701_CS_HIGH(J2_CS_GPIO_Port, J2_CS_Pin);
}

uint16_t MT6701_ReadRaw(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint8_t rx[2];

    MT6701_CS_LOW(cs_port, cs_pin);
    HAL_SPI_Receive(&hspi1, rx, 2, 100);
    MT6701_CS_HIGH(cs_port, cs_pin);

    uint16_t data = ((uint16_t)rx[0] << 8) | rx[1];
    data >>= 2;  // 14-bit 角度在 bits 15:2
    return data & 0x3FFF;
}

float MT6701_GetAngle(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint16_t raw = MT6701_ReadRaw(cs_port, cs_pin);
    return raw * 360.0f / 16384.0f;
}
```

### Step 3.2 — 舵机 PWM 封装

```c
// App/drivers/servo.h
#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

void Servo_Init(void);
uint16_t Servo_AngleToPulse(float angle_deg);  // 300° 角度 → 1000~5000
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg);

#endif
```

```c
// App/drivers/servo.c
#include "servo.h"
#include "tim.h"

// TIM4 配置: PSC=17, ARR=39999 → 每 tick = 0.5μs
// 500μs = 1000, 2500μs = 5000, 对应 0°~300°
// 脉宽计算：pulse = 1000 + (angle / 300.0) × 4000

#define SERVO_PULSE_MIN     1000    // 500μs → 0°
#define SERVO_PULSE_MAX     5000    // 2500μs → 300°
#define SERVO_ANGLE_MIN     0.0f
#define SERVO_ANGLE_MAX     300.0f

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // 初始设为 150°（中间位置 = 1500μs = 3000 ticks）
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 3000);  // J1 中间
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 3000);  // J2 中间
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);  // 夹爪1 全开
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);  // 夹爪2 全开
}

uint16_t Servo_AngleToPulse(float angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;

    // 300° 范围映射到 1000~5000
    return (uint16_t)(SERVO_PULSE_MIN +
        (angle_deg / SERVO_ANGLE_MAX) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
}

void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse)
{
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
{
    Servo_SetPulse(htim, channel, Servo_AngleToPulse(angle_deg));
}
```

### Step 3.3 — 实现舵机+编码器联合控制任务

```c
// App/tasks/servo_task.c
#include "app_includes.h"
#include "mt6701.h"
#include "servo.h"
#include "app_arm_config.h"

extern ArmState_t g_arm_state;
extern osMessageQueueId_t jointCmdQueueHandle;

void Servo_Task(void *argument)
{
    Servo_Init();
    MT6701_Init();

    Arm_JointCmd_t cmd;

    for (;;) {
        // 1. 检查命令（非阻塞）
        if (osMessageQueueGet(jointCmdQueueHandle, &cmd, NULL, 0) == osOK) {
            switch (cmd.joint_id) {
                case 1: g_arm_state.j1_target = cmd.value; break;  // J1 角度
                case 2: g_arm_state.j2_target = cmd.value; break;  // J2 角度
                case 3: g_arm_state.gripper_target = cmd.value; break; // 夹爪
            }
        }

        // 2. 读取 J1 MT6701 角度
        g_arm_state.j1_raw = MT6701_ReadRaw(J1_CS_GPIO_Port, J1_CS_Pin);
        g_arm_state.j1_deg = g_arm_state.j1_raw * 360.0f / 16384.0f;

        // 3. 读取 J2 MT6701 角度
        g_arm_state.j2_raw = MT6701_ReadRaw(J2_CS_GPIO_Port, J2_CS_Pin);
        g_arm_state.j2_deg = g_arm_state.j2_raw * 360.0f / 16384.0f;

        // 4. 舵机位置输出（直接将目标角度映射为 PWM 脉宽）
        Servo_SetAngle(&htim4, TIM_CHANNEL_1, g_arm_state.j1_target);
        Servo_SetAngle(&htim4, TIM_CHANNEL_2, g_arm_state.j2_target);
        // 夹爪：直接设脉宽（0=全开, 1000=全闭）
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, g_arm_state.gripper_target);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, g_arm_state.gripper_target);

        osDelay(20);  // 20ms 周期，约 50Hz 更新率
    }
}
```

### Step 3.4 — 单步测试

1. **测试 MT6701 读取**（不接舵机）：
   ```c
   // 在 servo_task 中只读不写
   printf("J1 raw=%u deg=%.1f  J2 raw=%u deg=%.1f\r\n",
          g_arm_state.j1_raw, g_arm_state.j1_deg,
          g_arm_state.j2_raw, g_arm_state.j2_deg);
   ```
   手动转动磁铁，观察角度输出。**验收标准**：✅ 0~16383 完整范围，平滑连续

2. **测试舵机 PWM**（不接编码器）：
   注释掉 MT6701 读取，只输出固定角度：
   ```c
   Servo_SetAngle(&htim4, TIM_CHANNEL_1, 90);  // J1 到 90°
   osDelay(1000);
   Servo_SetAngle(&htim4, TIM_CHANNEL_1, 0);    // J1 到 0°
   osDelay(1000);
   Servo_SetAngle(&htim4, TIM_CHANNEL_1, 180);  // J1 到 180°
   ```
   **验收标准**：✅ 舵机能转到指定角度

3. **闭环测试**：编码器读数 = 舵机实际输出角度
   ```c
   Servo_SetAngle(&htim4, TIM_CHANNEL_1, target_angle);
   float actual = MT6701_GetAngle(J1_CS_GPIO_Port, J1_CS_Pin);
   printf("target=%.0f actual=%.1f error=%.1f\r\n",
          target_angle, actual, target_angle - actual);
   ```
   **验收标准**：✅ 误差 < 5° 即满足（齿轮箱有回差，不需要 PID）

---

## Phase 5：CAN 命令控制 + 状态上报 — ⏳ 待开始

| 功能 | 状态 | 说明 |
|:----|:----:|:----|
| CAN 0x130 角度控制 | ✅ **已在 CAN_Rx_Task 实现** | JointCmdQueue + g_arm_state |
| CAN 0x430 回中/设速度 | ✅ 已在 CAN_Rx_Task 实现 | 0x01回中, 0x02设速度 |
| CAN 0x230 状态查询 | ⏳ | 待实现 |
| CAN 0x330 状态上报 | ⏳ | 待实现 Arm_State_Task |

### Step 5.1 — 实现 Arm_State_Task（50ms 上报）— ⏳ 未开始

```c
// App/tasks/arm_state_task.c
#include "app_includes.h"

extern ArmState_t g_arm_state;

void Arm_State_Task(void *argument)
{
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x330,
        .DLC = 8,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
    };
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    for (;;) {
        tx_data[0] = g_arm_state.j0_current_speed & 0xFF;
        tx_data[1] = (g_arm_state.j0_current_speed >> 8) & 0xFF;
        tx_data[2] = g_arm_state.j1_raw & 0xFF;
        tx_data[3] = (g_arm_state.j1_raw >> 8) & 0xFF;
        tx_data[4] = g_arm_state.j2_raw & 0xFF;
        tx_data[5] = (g_arm_state.j2_raw >> 8) & 0xFF;
        tx_data[6] = (uint8_t)g_arm_state.gripper_target;

        uint8_t flags = 0;
        if (g_arm_state.j0_stall)  flags |= 0x01;
        if (g_arm_state.j1_in_range) flags |= 0x02;
        if (g_arm_state.j2_in_range) flags |= 0x04;
        tx_data[7] = flags;

        HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);

        osDelay(50);  // 50ms = 20Hz
    }
}
```

### Step 5.2 — 全局状态结构体

```c
// App/config/app_globals.h
#ifndef __APP_GLOBALS_H__
#define __APP_GLOBALS_H__

#include "app_includes.h"

typedef struct {
    int16_t j0_target;           // J0 目标速度
    int16_t j0_current_speed;    // J0 当前速度
    int16_t j0_pwm;              // J0 PWM 输出值
    int32_t j0_ticks;            // J0 编码器差值
    int32_t j0_accumulated_ticks; // J0 累计编码器

    float   j1_target;           // J1 目标角度 (°)
    float   j1_deg;              // J1 当前角度 (°)
    uint16_t j1_raw;             // J1 MT6701 原始值

    float   j2_target;           // J2 目标角度 (°)
    float   j2_deg;              // J2 当前角度 (°)
    uint16_t j2_raw;             // J2 MT6701 原始值

    uint16_t gripper_target;     // 夹爪目标位置 (500~1000)

    uint8_t j0_stall;            // J0 堵转标志
    uint8_t j1_in_range;         // J1 到位标志
    uint8_t j2_in_range;         // J2 到位标志
} ArmState_t;

extern ArmState_t g_arm_state;

// 队列消息
typedef struct {
    uint8_t  joint_id;   // 0=J0, 1=J1, 2=J2, 3=Gripper, 0xFF=All
    int16_t  value;      // 目标值
    uint8_t  cmd;        // 0x11=SET, 0x08=STOP
} Arm_JointCmd_t;

extern osMessageQueueId_t jointCmdQueueHandle;

#endif
```

### Step 5.3 — CAN 命令解析（整合到 CAN_Rx_Task）

```c
// 在 CAN_Rx_Task 的 switch 中增加：
if (msg.id == 0x130) {
    // ARM_CMD 帧
    uint8_t cmd = msg.data[0];
    uint8_t joint_id = msg.data[1];
    int16_t value = (int16_t)(msg.data[2] | (msg.data[3] << 8));

    Arm_JointCmd_t jcmd = { .joint_id = joint_id, .value = value, .cmd = cmd };
    osMessageQueuePut(jointCmdQueueHandle, &jcmd, 0, 0);

    // 同时更新全局目标（确保未消费时也能读取最新值）
    switch (joint_id) {
        case 0: g_arm_state.j0_target = value; break;
        case 1: g_arm_state.j1_target = value; break;
        case 2: g_arm_state.j2_target = value; break;
        case 3: g_arm_state.gripper_target = value; break;
        case 0xFF:
            g_arm_state.j0_target = value;
            g_arm_state.j1_target = value;
            g_arm_state.j2_target = value;
            break;
    }
}
```

### Step 5.4 — 联调测试

**CAN 命令 → 机械臂动作：**

| 命令帧 (CAN ID 0x130) | 期望行为 |
|------------------------|---------|
| `11 00 32 00 00 00 00 00` | J0 以速度 50 正转 |
| `11 01 2D 00 00 00 00 00` | J1 转到 45° (0x002D=45) |
| `11 02 5A 00 00 00 00 00` | J2 转到 90° (0x005A=90) |
| `11 03 E8 03 00 00 00 00` | 夹爪关闭 (0x03E8=1000) |
| `08 FF 00 00 00 00 00 00` | 全关节停止 |

SavvyCAN 发送上述帧，观察各关节响应。同时收 `0x330` 状态帧，验证数据字段正确。

**验收标准**：✅ 每个关节独立受控，状态帧数据与实测一致

---

## Phase 6：ChassisController 网关适配 — ⏳ 待开始

> 最后一步：让底盘网关能转发机械臂命令。

### Step 6.1 — 网关增加 UART → CAN 机械臂命令

`5_ChassisController_t1/App/app_config.h`：
```c
// 新增命令枚举
#define CMD_ARM_CONTROL     0x10    // 机械臂控制
#define CMD_ARM_GET_STATE   0x11    // 查询机械臂状态
```

`5_ChassisController_t1/App/app_task.c`，`CommandProcess_Task_Run` 中：
```c
case CMD_ARM_CONTROL:
{
    // UART 帧: AA 10 [ID(4)] [DLC] [data...]
    // → CAN 帧: 0x130
    App_CAN_Message_t can_tx;
    can_tx.id = 0x130;
    can_tx.len = (uart_msg.len > 8) ? 8 : uart_msg.len;
    memcpy(can_tx.data, uart_msg.data, can_tx.len);
    osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0);
    break;
}

case CMD_ARM_GET_STATE:
{
    // 发送查询命令到 CAN 总线
    App_CAN_Message_t can_tx;
    can_tx.id = 0x230;
    can_tx.len = 0;
    osMessageQueuePut(canTxQueueHandle, &can_tx, 0, 0);
    break;
}
```

### Step 6.2 — 网关解析机械臂状态帧

`CanRxProcess_Task_Run` 中：
```c
if (std_id == 0x330) {
    // 解码机械臂状态
    g_system_state.arm.j0_speed = (int16_t)(data[0] | (data[1] << 8));
    g_system_state.arm.j1_angle_raw = (uint16_t)(data[2] | (data[3] << 8));
    g_system_state.arm.j2_angle_raw = (uint16_t)(data[4] | (data[5] << 8));
    g_system_state.arm.gripper_pos = data[6];
    g_system_state.arm.flags = data[7];
}
```

### Step 6.3 — 整车测试

上层（PC 串口）发送：
```
AA 10 30 01 00 00 08 11 00 32 00 00 00 00 00 00
```

期望：J0 电机以速度 50 旋转，机械臂状态通过 CAN `0x330` 上报给网关，网关可在 UART 回传状态。

**验收标准**：✅ 整车系统：PC → UART → 网关 → CAN → 机械臂控制器 → 各关节动作，状态反向流回

---

## 附录：常见调试问题

| 现象 | 可能原因 | 检查方法 |
|------|---------|---------|
| LED 不闪 | FreeRTOS 未启动 | 检查 TIM3 是否设为 timebase |
| CAN 收不到 | 波特率不匹配 | 示波器/逻辑分析仪看 CAN_H/L |
| MT6701 读 65535 | MISO 没接 | 检查接线 |
| MT6701 读数跳变 | SPI Mode 不对 | 确认 CPOL=Low, CPHA=2Edge |
| 舵机不动 | PWM 频率不是 50Hz | 检查 TIM4 Prescaler/ARR |
| J0 电机抖动 | PID 参数未整定 | 先调大 KP, 再加 KI |
| printf 不输出浮点 | 缺少 _printf_float | 加链接器标志 |
| 烧录失败 | JLink 路径不对 | 检查 flash.jlink |

---

## 快速参考：文件清单

实施完成后，项目应有以下文件：

```
3_SteeringArm/
├── Core/Src/
│   ├── main.c              # CubeMX + printf + 入口
│   └── freertos.c          # 任务创建 + 队列创建
├── App/
│   ├── config/
│   │   ├── app_config.h    # CAN ID + 全局参数
│   │   ├── app_globals.h   # ArmState_t + extern
│   │   ├── app_includes.h  # 统一头文件
│   │   └── app_arm_config.h # J0 电机/PID 参数
│   ├── tasks/
│   │   ├── can_rx_task.c   # CAN 接收 + 命令分发
│   │   ├── dc_motor_task.c # J0 PID 控制 (10ms)
│   │   ├── servo_task.c    # J1/J2/夹爪 PWM + MT6701 (20ms)
│   │   ├── arm_state_task.c # CAN 状态上报 (50ms)
│   │   └── heartbeat_task.c # LED 闪烁
│   ├── modules/
│   │   ├── pid.c/.h        # (复用 MCLM)
│   │   ├── filter.c/.h     # (复用 MCLM)
│   │   └── speed_map.c/.h  # (复用 MCLM)
│   ├── drivers/
│   │   ├── motor_DC_tb6612.c/.h # (复用 MCLM)
│   │   ├── servo.c/.h      # 舵机 PWM 封装
│   │   └── mt6701.c/.h     # MT6701 SPI 驱动
│   └── services/
│       ├── can_filter.c/.h # CAN ID 白名单
│       └── command.h       # App_CAN_Message_t
└── cmake/
    └── gcc-arm-none-eabi.cmake  # 工具链
```
