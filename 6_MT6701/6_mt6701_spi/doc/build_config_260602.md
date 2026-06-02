# MT6701 SSI (SPI) 读取 — 项目构建与烧录配置记录

> 日期: 2026-06-02
> MCU: STM32F103C8T6 (LQFP48)
> 目标: SPI1 读取 MT6701 角度，USART1 打印

---

## 目录

1. [CubeMX 配置](#一cubemx-配置)
2. [VS Code 工作区配置](#二vs-code-工作区配置)
3. [CMake 构建配置](#三cmake-构建配置)
4. [MT6701 驱动代码](#四mt6701-驱动代码)
5. [快捷烧录](#五快捷烧录)
6. [硬件接线](#六硬件接线)
7. [常见问题](#七常见问题)

---

## 一、CubeMX 配置

### 1.1 SPI1

路径: Pinout & Configuration → Connectivity → SPI1

| 参数 | 值 |
|------|-----|
| Mode | Full Duplex Master |
| Data Size | 8 Bits |
| First Bit | MSB First |
| Prescaler | 64 |
| CPOL | Low |
| CPHA | 2 Edge |
| NSS | Software |

引脚分配:
- PA5 → SPI1_SCK
- PA6 → SPI1_MISO
- PA7 → SPI1_MOSI (自动分配，硬件不接)

> **注意**: 虽然 MT6701 不需要 MOSI，但 HAL 用 Full Duplex Master 最省事，发空字节收数据。

### 1.2 USART1

路径: Pinout & Configuration → Connectivity → USART1

| 参数 | 值 |
|------|-----|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |

引脚分配:
- PA9 → USART1_TX
- PA10 → USART1_RX

### 1.3 PA4 GPIO

路径: Pinout & Configuration → GPIO → 点击 PA4

| 参数 | 值 |
|------|-----|
| GPIO mode | Output Push-Pull |
| Pin label | MT6701_CS |
| Default output level | High (GPIO_PIN_SET) |

### 1.4 时钟配置

- HSE (8MHz 外部晶振) → PLL x9 → 72MHz SYSCLK
- APB1: 36MHz (分频2)
- APB2: 72MHz (分频1)

---

## 二、VS Code 工作区配置

### 2.1 `.vscode/settings.json`

```json
{
  "cmake.cmakePath": "cube-cmake",
  "cmake.configureArgs": [
    "-DCMAKE_COMMAND=cube-cmake"
  ],
  "cmake.preferredGenerators": [
    "Ninja"
  ],
  "stm32cube-ide-clangd.path": "cube",
  "stm32cube-ide-clangd.arguments": [
    "starm-clangd",
    "--query-driver=${env:CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc*",
    "--query-driver=${env:CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-g++*"
  ]
}
```

> `cube-cmake` 由 STM32 VS Code 扩展提供，如果找不到则需重载窗口 (Reload Window) 等待扩展启动完成。

### 2.2 `.vscode/tasks.json`

```json
{
  "version": "2.0.0",
  "tasks": [
    {
        "label": "Build",
        "type": "shell",
        "command": "cmake --build ${workspaceFolder}/build/Debug",
        "problemMatcher": []
    },
    {
      "label": "Flash STM32 via J-Link",
      "type": "process",
      "command": "/opt/SEGGER/JLink/JLinkExe",
      "args": [
        "-Device", "STM32F103C8",
        "-If", "SWD",
        "-Speed", "4000",
        "-CommanderScript", "${workspaceFolder}/flash.jlink"
      ],
      "group": { "kind": "build", "isDefault": true },
      "problemMatcher": [],
      "dependsOn": "Build"
    }
  ]
}
```

- `Ctrl+Shift+B` 触发 Flash 任务（自动先 Build 再烧录）
- JLink Commander Script 指向项目根目录的 `flash.jlink`（含相对路径 `build/Debug/xxx.elf`）

### 2.3 `.vscode/c_cpp_properties.json`

```json
{
  "configurations": [
    {
      "name": "STM32",
      "compileCommands": "${workspaceFolder}/build/Debug/compile_commands.json"
    }
  ]
}
```

---

## 三、CMake 构建配置

### 3.1 `cmake/gcc-arm-none-eabi.cmake` — printf 浮点支持

默认 `--specs=nano.specs` 裁剪了 `printf` 的 `%f` 支持，需追加链接器标志:

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,_printf_float")
```

代价: Flash 占用从 ~32% 增加到 ~51%（约 +12KB）。

### 3.2 `flash.jlink.in` — JLink 脚本模板

由 cmake `configure_file()` 生成到 `build/Debug/flash.jlink`:

```
r
h
loadfile @CMAKE_PROJECT_NAME@.elf
r
q
```

### 3.3 `CMakeLists.txt` — flash 自定义目标

在文件末尾追加:

```cmake
# ===== One-click flash via J-Link =====
set(JLINK_SPEED "4000" CACHE STRING "J-Link SWD speed in kHz")
configure_file(${CMAKE_SOURCE_DIR}/flash.jlink.in ${CMAKE_BINARY_DIR}/flash.jlink @ONLY)

add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing ${CMAKE_PROJECT_NAME}..."
    COMMAND JLinkExe -Device STM32F103C8 -If SWD -Speed ${JLINK_SPEED}
        -AutoConnect 1 -CommanderScript ${CMAKE_BINARY_DIR}/flash.jlink
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Build + Flash ${CMAKE_PROJECT_NAME} via J-Link"
)
```

终端使用: `cmake --build build/Debug --target flash`

### 3.4 `flash.jlink`（项目根目录）

供 VS Code 任务使用，路径从项目根目录算起:

```
r
h
loadfile build/Debug/6_mt6701_spi.elf
r
q
```

> **注意**: 根目录的 `flash.jlink` 和 cmake 生成的 `build/Debug/flash.jlink` 是两个文件，路径写法不同，各有用途。

---

## 四、MT6701 驱动代码

全部写在 `Core/Src/main.c` 的 USER CODE 区域内，保证 CubeMX 重新生成代码时不被覆盖。

### 4.1 Includes

```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */
```

### 4.2 CS 控制宏

```c
/* USER CODE BEGIN PD */
#define MT6701_CS_LOW()  HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_RESET)
#define MT6701_CS_HIGH() HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_SET)
/* USER CODE END PD */
```

### 4.3 读取与转换函数 + printf 重定向

```c
/* USER CODE BEGIN 0 */

static uint16_t MT6701_ReadRaw(void)
{
    uint8_t rx[2];
    MT6701_CS_LOW();
    HAL_SPI_Receive(&hspi1, rx, 2, 100);
    MT6701_CS_HIGH();

    uint16_t data = ((uint16_t)rx[0] << 8) | rx[1];
    data >>= 2;
    return data & 0x3FFF;
}

static float MT6701_GetAngle(void)
{
    uint16_t raw = MT6701_ReadRaw();
    return raw * 360.0f / 16384.0f;
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* USER CODE END 0 */
```

### 4.4 主循环

```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      float angle = MT6701_GetAngle();
      printf("angle=%.2f\r\n", angle);
      HAL_Delay(100);
    /* USER CODE END WHILE */
```

---

## 五、快捷烧录

### 5.1 VS Code 一键烧录

`Ctrl+Shift+B` → 自动执行:
1. Build: `cmake --build build/Debug`
2. Flash: `JLinkExe -Device STM32F103C8 -If SWD -Speed 4000 -CommanderScript flash.jlink`

### 5.2 终端命令行烧录

```bash
cmake --build build/Debug --target flash
```

---

## 六、硬件接线

| MT6701 模块 | STM32 F103C8 | 备注 |
|------------|-------------|------|
| VCC | 3.3V | 不要接 5V |
| GND | GND | |
| NCS/Z | PA4 | CS 片选 |
| SCLK/B | PA5 | SPI1 SCK |
| MISO/A | PA6 | SPI1 MISO |
| NC | - | 悬空不接 |

> MOSI (PA7) 在 CubeMX 中自动分配但硬件不接。

---

## 七、常见问题

### 7.1 `cube-cmake` 找不到

```
Error: spawn cube-cmake ENOENT
```

**解决**: `Ctrl+Shift+P → Developer: Reload Window`，等待 STM32 扩展启动完成。

### 7.2 printf 只输出 `angle=` 没有数值

**原因**: newlib-nano 默认不链接 `%f` 支持。

**解决**: 在 `cmake/gcc-arm-none-eabi.cmake` 中添加:
```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-u,_printf_float")
```

### 7.3 JLink 烧录时报 `Failed to open file`

**原因**: `loadfile` 路径不正确。VS Code 任务从项目根目录启动，需要 `build/Debug/xxx.elf`。

**解决**: 根目录 `flash.jlink` 中使用相对路径: `loadfile build/Debug/6_mt6701_spi.elf`

### 7.4 输出一直为 0

检查 CS 是否被拉低。

### 7.5 输出一直为 65535

检查 MISO 接线。

### 7.6 角度范围异常（如只有 180~360 或 90~270）

CPOL/CPHA 配置错误。尝试交换 CPOL/CPHA 组合（MT6701 常见坑）。

---

## 八、编译资源占用

| 项目 | 大小 | 占比 |
|------|------|------|
| Flash | 33,448 B | 51% (64KB) |
| RAM | 3,248 B | 15.9% (20KB) |

> 开启 `_printf_float` 增加约 12KB Flash 占用。

---

## 参考

- [goal_260602.md](./goal_260602.md) — 原始目标文档
- [CSDN: STM32HAL库使用SPI读取MT6701传感器数据](https://blog.csdn.net/yaolei5/article/details/155005022)
- [CSDN: HAL STM32 SSI/SPI方式读取MT6701磁编码器获取角度例程](https://blog.csdn.net/weixin_42880082/article/details/137921359)
