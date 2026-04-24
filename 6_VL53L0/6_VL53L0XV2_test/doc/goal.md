好，这个我给你按**工程可落地流程**讲一遍：从接线 → CubeMX → HAL驱动 → 测距读取，全程按 STM32F103C8T6 + VL53L0X V2 来。

目标是：**插上就能测距离（mm）**

---

# 🧩 一、硬件接线（先把这个定死）

## ⚡ 推荐标准接法（单模块）

| VL53L0X | STM32F103C8T6 | 说明          |
| ------- | ------------- | ----------- |
| VIN     | 3.3V          | 推荐3.3V供电    |
| GND     | GND           | 共地          |
| SCL     | PB6           | I2C1_SCL    |
| SDA     | PB7           | I2C1_SDA    |
| XSHUT   | PA1           | 使能控制（强烈建议接） |
| GPIO1   | 不接            | 先忽略         |

---

## ⚠️ 硬件关键点

### ✔ I2C必须上拉

* 4.7kΩ → SCL
* 4.7kΩ → SDA

👉 如果你的模块自带上拉，可以不加

---

# 🏗 二、CubeMX配置（重点）

## 1️⃣ 选芯片

```
STM32F103C8Tx
```

---

## 2️⃣ RCC时钟

推荐：

```
HSE = 8MHz
SYSCLK = 72MHz
```

---

## 3️⃣ I2C配置（核心）

### 🔧 I2C1

| 参数          | 设置                 |
| ----------- | ------------------ |
| Mode        | I2C                |
| Speed       | 100kHz（先稳，别上来400k） |
| Duty        | Standard           |
| Own Address | 0                  |

### 引脚自动配置：

* PB6 → I2C1_SCL
* PB7 → I2C1_SDA

---

## 4️⃣ GPIO配置

### XSHUT（PA1）

```
PA1 → GPIO_Output
```

配置：

* Push-Pull
* No Pull
* Speed Low

---

## 5️⃣ Debug

```
Serial Wire (SWD)
```

---

## 6️⃣ 生成工程

选择：

* MDK / Makefile / STM32CubeIDE 都行

---

# ⚙️ 三、上电逻辑（非常关键）

VL53L0X默认是“休眠状态逻辑敏感”的。

## 初始化必须这样做：

### ✔ XSHUT拉高

```c
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
HAL_Delay(10);
```

👉 作用：启动传感器

---

# 📦 四、驱动准备（最重要）

你需要 ST 官方 VL53L0X driver：

👉 关键词：

```
VL53L0X ULD driver ST
```

常见文件：

```
vl53l0x_api.c
vl53l0x_api.h
platform.c
```

---

# 🧠 五、最小可运行流程（核心逻辑）

## 1️⃣ 初始化I2C + 传感器

```c
VL53L0X_Dev_t dev;
VL53L0X_RangingMeasurementData_t measure;
```

---

## 2️⃣ 初始化流程

```c
VL53L0X_DataInit(&dev);
VL53L0X_StaticInit(&dev);

VL53L0X_SetDeviceMode(&dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
VL53L0X_StartMeasurement(&dev);
```

---

## 3️⃣ 循环读取

```c
while (1)
{
    VL53L0X_GetRangingMeasurementData(&dev, &measure);

    uint16_t distance = measure.RangeMilliMeter;

    printf("dist = %d mm\r\n", distance);

    VL53L0X_ClearInterruptMask(&dev, 0);
    HAL_Delay(50);
}
```

---

# ⚠️ 六、CubeMX + HAL必须补的坑

## ❌ 1. I2C不稳定（最常见）

解决：

* 降到 100kHz
* 确保上拉电阻
* SDA/SCL不要太长

---

## ❌ 2. 读不到设备

检查：

* XSHUT是否拉高
* 电压是否3.3V稳定
* 是否共地

---

## ❌ 3. 卡在初始化

通常是：

* I2C未初始化成功
* HAL库时序问题

👉 可以先加：

```c
HAL_Delay(100);
```

---

# 🚀 七、建议升级路线（适合你现在机器人）

你这个已经是底盘级别项目了，我建议你后面这样升级：

## 🔥 Step 1（现在）

✔ 单VL53L0X测距

---

## 🔥 Step 2（下一步）

✔ 4个环绕底盘
✔ XSHUT分时初始化
✔ 改I2C地址

---

## 🔥 Step 3（系统级）

✔ CAN节点发布距离
✔ ROS2 range topic
✔ EKF融合避障

---

# 🧠 八、如果你要做“工程级版本”

我可以帮你直接补：

### ✔ STM32完整工程模板

* CubeMX工程结构
* HAL驱动封装
* 非阻塞读取
* CAN发布版本

### ✔ 多VL53L0X自动地址分配代码

（这个是机器人项目关键）

---

如果你下一步说一句：
👉 “我要多VL53L0X + CAN上报 + ROS2”

我可以直接帮你搭一个**底盘级防撞系统架构**。
