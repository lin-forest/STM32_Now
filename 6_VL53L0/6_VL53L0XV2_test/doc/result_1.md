# VL53L0X V2 集成方案（STM32F103C8T6 + HAL）

## 一、项目现状

| 项目 | 内容 |
|------|------|
| 芯片 | STM32F103C8T6（72 MHz，HSE×9 PLL）|
| IDE/构建 | STM32CubeIDE → CMake（cmake/stm32cubemx/CMakeLists.txt）|
| I2C 总线 | I2C1，PB6=SCL，PB7=SDA，400 kHz Fast-mode |
| XSHUT 引脚 | PA1，GPIO Output，默认高电平 |
| 现有 main.c | 仅 HAL_Init + 时钟 + MX_GPIO_Init + MX_I2C1_Init，while(1) 为空 |

---

## 二、要新增的文件

```
Drivers/VL53L0X/
├── Inc/
│   ├── vl53l0x_platform.h   ← HAL I2C 读写声明      【已创建】
│   └── vl53l0x.h            ← 驱动公开 API + 寄存器宏 【已创建】
└── Src/
    ├── vl53l0x_platform.c   ← HAL_I2C_Mem_Write/Read 封装 【已创建】
    └── vl53l0x.c            ← 驱动核心（Init/Ranging）     【待写】
```

修改的文件：

```
Core/Src/main.c                         ← 加入初始化 + 测距循环  【待写】
cmake/stm32cubemx/CMakeLists.txt        ← 加入驱动源文件 + Inc 路径 【待写】
```

---

## 三、驱动设计（vl53l0x.c）

### 3.1 Init 流程（严格按 ST AN4545 / Pololu 开源驱动顺序）

```
XSHUT 低→延时→高（硬复位）
等待 I2C 就绪（轮询 IDENTIFICATION_MODEL_ID = 0xEE）
写入 VHV_CONFIG_PAD 使能 2.8 V I/O
SPAD 管理：
  ① 读取 SPAD 数量/类型（NVM 引导值）
  ② 写入 REF_EN_START_SELECT
  ③ 使能对应 SPAD 位图
VHV / PhaseCal 参考校准
加载 ST 标准默认寄存器序列（来自 AN4545 附录）
设置 GPIO 中断（新数据就绪 → 低电平有效）
设置默认 Timing Budget = 33 ms
保存 stop_variable（0x91 读取）
```

### 3.2 Ranging 流程（连续模式）

```
StartContinuous(period_ms)
  ↓ 写 0x80=0x01, 0xFF=0x01, 0x00=0x00
  ↓ 写 stop_variable → 0x91
  ↓ 写 0x00=0x01, 0xFF=0x00, 0x80=0x00
  ↓ 若 period_ms > 0：写 INTERMEASUREMENT_PERIOD
  ↓ 写 SYSRANGE_START = 0x04（周期）或 0x02（背靠背）

ReadRangeContinuousMillimeters()
  ↓ 轮询 RESULT_INTERRUPT_STATUS[2:0] != 0（超时 1 s）
  ↓ 读取 RESULT_RANGE_STATUS + 10（2 字节，大端）
  ↓ 写 SYSTEM_INTERRUPT_CLEAR = 0x01
  ↓ 返回 range_mm

StopContinuous()
  ↓ 写 SYSRANGE_START = 0x01
  ↓ 写恢复序列（0x80/0xFF/0x00）
```

### 3.3 Long-Range 模式（可选，最远 ~2 m）

```
SetVcselPulsePeriod(PRE_RANGE,  18)
SetVcselPulsePeriod(FINAL_RANGE, 14)
SetMeasurementTimingBudget(200000)   // 200 ms
```

---

## 四、main.c 修改点

```c
/* USER CODE BEGIN Includes */
#include "vl53l0x.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static VL53L0X_Dev_t  vl53;
static uint16_t       range_mm;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
// 1. XSHUT 硬复位（PA1 低→延时→高）
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
HAL_Delay(10);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
HAL_Delay(10);

// 2. 初始化传感器
if (VL53L0X_Init(&vl53, VL53L0X_I2C_ADDR_DEFAULT) != VL53L0X_OK) {
    Error_Handler();
}

// 3. （可选）Long-Range 模式
// VL53L0X_SetLongRangeMode(&vl53);

// 4. 启动连续测距（背靠背，period_ms = 0）
VL53L0X_StartContinuous(&vl53, 0);
/* USER CODE END 2 */

while (1) {
    /* USER CODE BEGIN 3 */
    if (VL53L0X_ReadRangeContinuousMillimeters(&vl53, &range_mm) == VL53L0X_OK) {
        if (VL53L0X_IsOutOfRange(range_mm)) {
            // 超量程处理（> ~1.2 m 默认模式）
        } else {
            // range_mm 即为距离（mm）
        }
    }
    /* USER CODE END 3 */
}
```

---

## 五、CMakeLists.txt 修改点

在 `cmake/stm32cubemx/CMakeLists.txt` 中：

**① MX_Include_Dirs 追加：**
```cmake
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/VL53L0X/Inc
```

**② MX_Application_Src 追加：**
```cmake
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/VL53L0X/Src/vl53l0x.c
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/VL53L0X/Src/vl53l0x_platform.c
```

---

## 六、执行顺序

- [x] `Drivers/VL53L0X/Inc/vl53l0x_platform.h` — 已创建
- [x] `Drivers/VL53L0X/Inc/vl53l0x.h` — 已创建
- [x] `Drivers/VL53L0X/Src/vl53l0x_platform.c` — 已创建
- [ ] `Drivers/VL53L0X/Src/vl53l0x.c` — **下一步**
- [ ] `Core/Src/main.c` — 填写 USER CODE 区域
- [ ] `cmake/stm32cubemx/CMakeLists.txt` — 追加源文件和头文件路径

---

## 七、关键注意事项

| 问题 | 说明 |
|------|------|
| I2C 地址 | 7-bit = 0x29，HAL 调用时需 `<<1` 即 0x52 |
| 上拉电阻 | SCL/SDA 需 4.7 kΩ 上拉（模块自带则无需外加）|
| XSHUT 必须接 | 不接则无法硬复位，多传感器场景必须接 |
| 寄存器大端序 | VL53L0X 16-bit 寄存器全部大端，platform 层已处理 |
| 超量程判断 | range_mm ≥ 8190 表示超出测量范围，需软件判断 |
| Timing Budget | 默认 33 ms；精度要求高用 200 ms；速度优先用 20 ms |
