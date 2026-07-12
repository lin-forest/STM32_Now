# 3_SteeringArm 项目状态

> 最后更新：2026-07-14
> 分支：fix_base

---

## 一、已完成功能

### 基础框架
- [x] CubeMX 工程生成（STM32F103C8T6, 72MHz, FreeRTOS CMSIS-V2）
- [x] 时钟：HSE 8MHz → PLL×9 → 72MHz, APB1=36MHz, APB2=72MHz
- [x] TIM3 作为 HAL 时间基准（`Project Settings → HAL Settings → Timebase Source = TIM3`）
- [x] CMake 构建系统（gcc-arm-none-eabi + Makefile）
- [x] printf 重定向到 USART1 (115200)

### 通信
- [x] CAN1：500kbps, RX0+SCE 中断
- [x] CAN 硬件滤波：Bank0=0x130+0x131, Bank1=0x230, Bank2=0x430 (32bit mask mode)
- [x] CAN 0x130：ARM_CMD 角度控制（0x11 绝对 / 0x12 多关节 / 0x21 增量）
- [x] CAN 0x430：ARM_CONFIG 回中(0x01) / 设速度(0x02) / 锁定(0x03) / 解锁(0x04)
- [x] CAN 0x131：ARM_VEL 每关节独立速度
- [x] CAN 0x230：ARM_QUERY 状态查询 -> 触发 0x330 上报
- [x] CAN 0x330：ARM_STATUS 周期状态上报 (50ms, arm_state_task)
- [x] printf 调试输出

### 舵机控制
- [x] TIM4 50Hz PWM（PSC=35, ARR=39999, ⚠️ 翻倍后时钟=72MHz）
- [x] 舵机驱动层（App/drivers/servo.c/h）
- [x] 平滑插值（speed_dps, 20ms 周期）
- [x] 上电不输出角度（g_servo_active 标志，收到 CAN 命令后激活）
- [x] 默认速度 180°/s，可通过 0x430 动态调整
- [x] R2: J1(TIM4_CH1) / J2(TIM4_CH2) / J3 夹爪(TIM4_CH3+CH4)
- [x] R1: 双夹爪同步+独立控制 (CH1/CH2)
- [x] 串口输出实时 PWM CCR
- [x] J1/J2 软件限位（`J1_ANGLE_MIN/MAX` 在 can_rx_task 三模式中生效）
- [x] J3 增量控制（cmd=0x21, joint_id=3）
- [x] 多关节 J3 映射修复（uint8 0~200 → CCR 5000~3000）
- [x] 初始位置安全中位（`gripper_target=3000`，防零值抱死）

### MT6701 驱动
- [x] App/drivers/mt6701.c/h — SPI1 Mode1 读取 14-bit 角度
- [x] 双 CS 分时访问（J1=PB1, J2=PB12）

---

## 二、FreeRTOS 任务

| 任务 | 入口 | 优先级 | 栈(words) | 周期 | 当前状态 |
|:----|:----|:-----:|:---------:|:----:|:--------:|
| Heartbeat_Ta | `Heartbeat_Task_Run` | Low (8) | 64 | 300ms | ✅ LED 闪烁 |
| CAN_Rx_Ta | `CAN_Rx_Task_Run` | Normal (24) | 256 | 事件驱动 | ✅ 解析 0x130/0x230/0x430 + 白名单 |
| **Servo_Ta** | **`Servo_Task_Run`** | **Normal (24)** | **128** | **20ms** | **✅ 平滑+输出, J1/J2/J3 PWM** |
| DC_Motor_Ta | `DC_Motor_Task_Run` | Normal1 (25) | 256 | — | ⏳ 空等 |
| Arm_State_Ta | `Arm_State_Task_Run` | Normal (24) | 256 | **50ms** | **✅ 0x330 状态上报** |

### 任务间通信

```
CAN ISR (stm32f1xx_it.c)
  │  osMessageQueuePut(canRxQueueHandle)
  ▼
CAN_Rx_Task (freertos.c)
  │  解析 0x130/0x430 → 更新 g_arm_state
  │  激活 g_servo_active = 1
  ▼
Servo_Task (freertos.c)
  │  平滑插值 current → target
  │  Servo_SetAngle() → TIM4 输出
  ▼
舵机物理运动
```

---

## 三、全局状态结构体

```c
// App/config/app_globals.h
typedef struct {
    /* J1 舵机 */
    float   j1_target;          // 目标角度 (°), -150~+150
    float   j1_current;         // 当前插值角度
    uint16_t j1_raw;            // MT6701 原始值 0~16383

    /* J2 舵机 */
    float   j2_target;
    float   j2_current;
    uint16_t j2_raw;

    /* 角速度 */
    float   j1_speed_dps;       // 角速度 (°/s)
    float   j2_speed_dps;

    /* J3 夹爪 */
    uint16_t gripper_target;    // 脉宽 1000~5000

    /* R1 独立控制 */
    uint16_t ch1_target;        // 左舵机独立目标 (0=跟随 gripper_target)
    uint16_t ch2_target;        // 右舵机独立目标

    /* J0 预留 */
    int16_t j0_target;
    int16_t j0_current_speed;
    uint16_t j0_encoder_raw;
} ArmState_t;

extern ArmState_t g_arm_state;
extern volatile uint8_t g_servo_active;   // 0=不动, 1=激活
extern osMessageQueueId_t canRxQueueHandle;
```

---

## 四、CAN 协议

### 0x130 — ARM_CMD 关节控制

```
Byte 0:     0x11 = SET_POSITION (绝对)
            0x12 = MULTI (多关节同步)
            0x21 = INCREMENT (增量)
Byte 1:     关节ID (0=J0, 1=J1, 2=J2, 3=J3 Gripper) / bitmask(MULTI)
Byte 2..3:  目标值 (int16 LE, 0.1°精度)
Byte 4..7:  保留 / 多关节扩展数据
```

### 0x131 — ARM_VEL 每关节独立速度

```
Byte 0:     joint_id (1=J1, 2=J2, 0xFF=ALL)
Byte 1:     保留
Byte 2..3:  speed (uint16 LE, 0.1°/s)
```

### 0x230 — ARM_QUERY 状态查询

```
Byte 0:     0x01=QUERY_STATUS, 0x04=LOG_START, 0x05=LOG_STOP
```

### 0x330 — ARM_STATUS 状态上报 (TX, 50ms)

```
Byte 0..1:  J1 raw angle (uint16 LE, 0~16383, MT6701)
Byte 2..3:  J2 raw angle (uint16 LE, 0~16383)
Byte 4:     gripper_target / 200 (uint8, 0~25)
Byte 5:     J0 current speed (int8, 预留)
Byte 6:     flags: bit0=J1_IN_RANGE, bit1=J2_IN_RANGE
Byte 7:     0x00
```

### 0x430 — ARM_CONFIG 参数配置

```
data[0]=0x01:  J1/J2 回中 (0°)
data[0]=0x02 + data[1..2]=速度×10 (int16 LE):  设置 J1/J2 角速度
data[0]=0x03:  LOCK — g_servo_active=0, 停止 PWM 输出
data[0]=0x04:  UNLOCK — g_servo_active=1, 恢复 PWM 输出
```

### CAN ID 分配

| ID | 方向 | 功能 | 状态 |
|:--:|:----:|------|:----:|
| 0x130 | Chassis→Arm | 关节角度控制 (SET/MULTI/INCREMENT) | ✅ |
| 0x131 | Chassis→Arm | 每关节独立速度设置 | ✅ |
| 0x230 | Chassis→Arm | 状态查询 | ✅ |
| 0x330 | Arm→Chassis | 状态上报 (50ms) | ✅ |
| 0x430 | Chassis→Arm | 参数配置 (HOME/SPEED/LOCK/UNLOCK) | ✅ |

---

## 五、使用方式

### 命令行控制
```bash
python3 tools/can_angle.py           # 交互式计算器
python3 tools/can_angle.py --help    # 查看用法

# 或者直接用 cansend
cansend can0 130#1101840300000000     # J1→90°
cansend can0 430#0100000000000000     # 回中
cansend can0 430#02B80B00000000       # 速度 300°/s
```

### 编译烧录
```bash
cd 3_SteeringArm_t1
rm -rf build
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
make -C build -j$(nproc)
JLinkExe -Device STM32F103C8 -If SWD -Speed 4000 -CommanderScript flash.jlink
```

---

## 六、待实现任务

### P0 — MT6701 接线测试
- [ ] 接 J1/J2 MT6701 到 SPI1 总线
- [ ] 验证双 CS 分时读取
- [ ] 校准齿隙和安装偏移

### P1 — Phase 4：J0 直流电机
- [ ] 移植 TB6612 驱动 + PID（来自 3_MCLM_t2）
- [ ] 配置 TIM1 PWM + TIM2 编码器
- [ ] 实现速度内环（10ms） + 位置外环（50ms）
- [ ] J0 串级控制 + 齿隙补偿

### P1 — Phase 6：ChassisController 网关适配
- [ ] 增加 UART 命令 CMD_ARM_CONTROL(0x10)
- [ ] CommandProcess_Task 转发到 CAN 0x130
- [ ] CanRxProcess_Task 解析 0x330 状态

### P2 — 软件白名单
- [ ] App/services/can_filter.c 第二层命令字节校验

---

## 七、关键注意事项

### TIM4 时钟翻倍（踩坑记录）
STM32F1 的 APB1 定时器时钟规则：
- APB1 = 36MHz (72MHz/2)
- APB1 分频 ≠ 1 → 定时器时钟 = 36MHz × **2** = **72MHz**
- 所以 TIM4 PSC=35 → 72/36=2MHz → tick=0.5μs → 50Hz ✅
- 详见 `doc/tim4_clock_fix.md`

### 引脚冲突
| 引脚 | 功能 | 注意 |
|:----:|:----|:----|
| PA8 | TIM1_CH1 (J0 PWM) | 唯一无冲突的 PWM 通道 |
| PA9 | USART1_TX | 被 TIM1_CH2 占用，不能用于其他定时器 |
| PA11 | CAN1_RX | 被 TIM1_CH4 占用 |
| PB6~PB9 | TIM4_CH1~CH4 | **唯一 4 通道全空闲的定时器** |

### CubeMX 重生成注意事项
- `tim.c`：TIM4 的 PSC=35, ARR=39999 可能被覆盖，重生成后检查
- `freertos.c`：所有代码在 USER CODE 区域，不会被覆盖
- `main.h`：GPIO label 宏定义在此，重生成后确认 `J1_CS_Pin` 等存在
- **HAL Timebase** 每次重生成后检查：`Project Settings→HAL Settings→Timebase Source = TIM3`
