# STM32 快捷烧录配置 — 统一整理

## 一、流程说明

以 `5_ChassisController_t1` 为参考，统一为工作区中所有 STM32 CMake 项目配置 **一键编译+烧录**（通过 J-Link / SWD）。

### 配置涉及的 3 个层面

1. **CMakeLists.txt** → 定义 `flash` 编译目标（`add_custom_target(flash ...)`)
2. **flash.jlink.in** → J-Link 烧录脚本模板（`@CMAKE_PROJECT_NAME@.elf`）
3. **.vscode/tasks.json** → VSCode 任务定义（工作区级 + 项目级）

### 烧录原理

```
CMake 编译 → 生成 .elf → JLinkExe 通过 SWD 写入芯片
```

- 编译：`cmake --build build/Debug`
- 烧录：`cmake --build build/Debug --target flash`（自动先编译）
- 工具链：`arm-none-eabi-gcc` → `JLinkExe` → `STM32F103C8`

---

## 二、新增/修改文件清单

### 1. CMakeLists.txt — 新增 flash 目标（8 个项目）

所有项目统一在末尾追加以下代码块：

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

| # | 项目路径 | 状态 |
|---|---|---|
| 1 | `3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/CMakeLists.txt` | ✅ 已有 |
| 2 | `5_Tec_USART/5_ChassisController_t1/CMakeLists.txt` | ✅ 已有（参考源） |
| 3 | `3_SteeringArm/CMakeLists.txt` | ✅ **2025-06-23 新增** |
| 4 | `6_Mpu6050/6_Mpu6050_t1/CMakeLists.txt` | ✅ 新增 |
| 5 | `6_BMP280/6_BMP280_t1/CMakeLists.txt` | ✅ 新增 |
| 6 | `6_VL53L0/6_VL53L0XV2_test/CMakeLists.txt` | ✅ 新增 |
| 7 | `6_MT6701/6_MT6701_test/CMakeLists.txt` | ✅ 新增 |
| 8 | `6_LED_PC13_/6_CmakeForVsc_LedTest/CMakeLists.txt` | ✅ 新增 |

### 2. flash.jlink.in — 烧录脚本模板（6 个文件新增）

模板内容：
```
r
h
loadfile @CMAKE_PROJECT_NAME@.elf
r
q
```

由 `configure_file()` 在编译时生成到 `build/Debug/flash.jlink`。

| # | 文件路径 | 状态 |
|---|---|---|
| 1 | `3_MCLM_t2/flash.jlink.in` | ✅ 已有 |
| 2 | `5_ChassisController_t1/flash.jlink.in` | ✅ 已有 |
| 3 | `3_SteeringArm/flash.jlink.in` | ✅ **2025-06-23 新增** |
| 4 | `6_Mpu6050_t1/flash.jlink.in` | ✅ 新增 |
| 5 | `6_BMP280_t1/flash.jlink.in` | ✅ 新增 |
| 6 | `6_VL53L0XV2_test/flash.jlink.in` | ✅ 新增 |
| 7 | `6_MT6701_test/flash.jlink.in` | ✅ 新增 |
| 8 | `6_CmakeForVsc_LedTest/flash.jlink.in` | ✅ 新增 |

### 3. flash.jlink — 硬编码烧录脚本（供项目级 tasks.json 直接调用）

| # | 文件路径 | 状态 |
|---|---|---|
| 1 | `3_SteeringArm/flash.jlink` | ✅ **2025-06-23 新增** |
| 2 | `6_CmakeForVsc_LedTest/flash.jlink` | ✅ 新增 |
| 3 | `6_VL53L0XV2_test/flash.jlink` | ✅ **修复**（见下方 Bug） |

### 4. 工作区级 tasks.json — `0_Workspace/.vscode/tasks.json`

原有 5 个任务 → **扩展为 17 个任务**（2025-06-23 再加入 SteeringArm）：

**Build 任务（8个）：**
- `Build: 3_MCLM_t2`
- `Build: ChassisController`
- `Build: SteeringArm`
- `Build: MPU6050`
- `Build: BMP280`
- `Build: VL53L0`
- `Build: MT6701`
- `Build: LED`

**Flash 任务（8个）：**
- `Flash: 3_MCLM_t2`（依赖 `Build: 3_MCLM_t2`）
- `Flash: ChassisController`（依赖 `Build: ChassisController`）
- `Flash: SteeringArm`（依赖 `Build: SteeringArm`）
- `Flash: MPU6050`（依赖 `Build: MPU6050`）
- `Flash: BMP280`（依赖 `Build: BMP280`）
- `Flash: VL53L0`（依赖 `Build: VL53L0`）
- `Flash: MT6701`（依赖 `Build: MT6701`）
- `Flash: LED`（依赖 `Build: LED`）

**一键全烧任务（1个）：**
- `Flash: All Nodes` → 依次烧录全部 8 个项目

### 5. 项目级 tasks.json — `.vscode/tasks.json`

每个项目有两级 `.vscode/tasks.json`：

**① 子目录级（直接打开项目文件夹时生效）：**

| # | 位置 | 状态 |
|---|---|---|
| 1 | `3_MCLM_t2/.vscode/tasks.json` | ✅ 已有 |
| 2 | `5_ChassisController_t1/.vscode/tasks.json` | ✅ 已有 |
| 3 | `3_SteeringArm/.vscode/tasks.json` | ✅ **2025-06-23 新增** |
| 4 | `6_Mpu6050_t1/.vscode/tasks.json` | ✅ 已有 |
| 5 | `6_BMP280_t1/.vscode/tasks.json` | ✅ 已有 |
| 6 | `6_VL53L0XV2_test/.vscode/tasks.json` | ✅ 已有 |
| 7 | `6_MT6701_test/.vscode/tasks.json` | ✅ 已有 |
| 8 | `6_CmakeForVsc_LedTest/.vscode/tasks.json` | ✅ 新增 |

**② 工作区文件夹根目录级（通过 `robot.code-workspace` 打开时 Ctrl+Shift+B 可见）：**

> **关键：** VSCode 多根工作区中，Ctrl+Shift+B 只读取每个工作区文件夹**根目录**下的 `.vscode/tasks.json`。传感器项目的实际 CMake 目录比工作区文件夹深一层，因此需要在根目录补一份。
>
> **注意：** 这些任务的 Flash 步骤必须用 `"options": {"cwd": "<子目录>"}` 指定工作目录，否则 JLinkExe 会从父目录查找 `build/Debug/xxx.elf` 而找不到文件。

| # | 工作区文件夹 | 实际项目 | 状态 |
|---|---|---|---|
| 1 | `6_Mpu6050/.vscode/tasks.json` | → `6_Mpu6050_t1/` | ✅ **新增** |
| 2 | `6_BMP280/.vscode/tasks.json` | → `6_BMP280_t1/` | ✅ **新增** |
| 3 | `6_VL53L0/.vscode/tasks.json` | → `6_VL53L0XV2_test/` | ✅ **新增** |
| 4 | `6_MT6701/.vscode/tasks.json` | → `6_MT6701_test/` | ✅ **新增** |
| 5 | `6_LED_PC13_/.vscode/tasks.json` | → `6_CmakeForVsc_LedTest/` | ✅ **新增** |

---

## 三、🐛 修复的 Bug

### VL53L0 flash.jlink 引用错误的 .elf 文件

**文件**: `6_VL53L0/6_VL53L0XV2_test/flash.jlink`

- **错误**: `loadfile build/Debug/3_MCLM_t2.elf`（复制粘贴错误）
- **修正**: `loadfile build/Debug/6_VL53L0XV2_test.elf`

---

## 四、使用方法

### 方式 1：VSCode 任务（推荐）

打开 `robot.code-workspace` → `终端` → `运行任务...` → 选择：

| 任务 | 说明 |
|---|---|
| `Build: <项目名>` | 仅编译 |
| `Flash: <项目名>` | 编译 + 烧录指定项目 |
| `Flash: All Nodes` | 编译 + 烧录全部项目 |

### 方式 2：命令行

```bash
# 编译+烧录指定项目
cmake --build <项目路径>/build/Debug --target flash

# 示例
cmake --build 6_Mpu6050/6_Mpu6050_t1/build/Debug --target flash
```

### 方式 3：直接调用 JLink（项目目录下）

```bash
JLinkExe -Device STM32F103C8 -If SWD -Speed 4000 -CommanderScript flash.jlink
```

---

## 五、换用 F4 系列（如 STM32F407VG）的迁移要点

当前配置全部面向 **STM32F103C8**（Cortex-M3，64KB Flash/20KB RAM）。
换用 F4（Cortex-M4，带 FPU）时，以下所有层面都需要修改：

### 修改清单

| # | 修改项 | 当前值 (F1) | F4 范例 (F407VG) | 涉及文件数 |
|---|---|---|---|---|
| 1 | `-Device` JLink 型号 | `STM32F103C8` | `STM32F407VG` | **20 处** — CMakeLists.txt ×8 + 子目录 tasks.json ×8 + 根目录 tasks.json ×5 |
| 2 | 链接脚本 | `STM32F103XX_FLASH.ld` | `STM32F407VGTx_FLASH.ld` | 每个项目 1 个 .ld |
| 3 | 启动文件 | `startup_stm32f103xb.s` | `startup_stm32f407xx.s` | CubeMX 生成 |
| 4 | CMake CPU 标志 | `-mcpu=cortex-m3` | `-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16` | `cmake/gcc-arm-none-eabi.cmake` |
| 5 | SWD 速度 | 2000~4000 kHz | 可达 8000~12000 kHz | CMakeLists.txt `JLINK_SPEED` |
| 6 | Flash 扇区 | 1KB 统一页 | 前 4×16KB + 后 64KB/128KB | JLink 自动根据 `-Device` 选择 |

### 当前所有 `-Device STM32F103C8` 硬编码位置

**CMakeLists.txt（8 处）：**
```
3_MCLM_t2/CMakeLists.txt
5_ChassisController_t1/CMakeLists.txt
3_SteeringArm/CMakeLists.txt
6_Mpu6050_t1/CMakeLists.txt
6_BMP280_t1/CMakeLists.txt
6_VL53L0XV2_test/CMakeLists.txt
6_MT6701_test/CMakeLists.txt
6_CmakeForVsc_LedTest/CMakeLists.txt
```

**子目录级 tasks.json（8 处）：**
```
3_MCLM_t2/.vscode/tasks.json
5_ChassisController_t1/.vscode/tasks.json
3_SteeringArm/.vscode/tasks.json
6_Mpu6050_t1/.vscode/tasks.json
6_BMP280_t1/.vscode/tasks.json
6_VL53L0XV2_test/.vscode/tasks.json
6_MT6701_test/.vscode/tasks.json
6_CmakeForVsc_LedTest/.vscode/tasks.json
```

**工作区文件夹根目录级 tasks.json（5 处）：**
```
6_Mpu6050/.vscode/tasks.json
6_BMP280/.vscode/tasks.json
6_VL53L0/.vscode/tasks.json
6_MT6701/.vscode/tasks.json
6_LED_PC13_/.vscode/tasks.json
```

### 需要改的地方

**① CMakeLists.txt** — 目前 `-Device STM32F103C8` 是**硬编码**（只有 `JLINK_SPEED` 是缓存变量）：

```cmake
COMMAND JLinkExe -Device STM32F103C8  # ← 硬编码，需手动改为 F4 型号
```

迁移时建议顺便把 Device 也抽成变量：

```cmake
set(JLINK_DEVICE "STM32F407VG" CACHE STRING "J-Link device name")
...
COMMAND JLinkExe -Device ${JLINK_DEVICE} ...
```

**② tasks.json 中写死的 13 处 `-Device`**（8 子目录 + 5 根目录）同样需手动更新。

### 最优方案

tasks.json 全部改用 `cmake --build ... --target flash` 间接调用，不直接写 JLinkExe。
这样只需在 **CMakeLists.txt 一个地方**维护 Device 型号。

### 风险警示

- **Flash 算法不匹配**：F1 算法擦 F4 → 报错或锁芯片（RDP/Option Bytes 损坏）
- **.elf 超尺寸**：F1 的 .ld 只有 64KB Flash，F4 固件可能 >64KB → 链接报错
- **FPU 未启用**：F4 的 M4 核若不加 `-mfloat-abi=hard`，浮点运算退化成软浮点，体积大且慢

## 六、新建项目配置指南（以 3_SteeringArm 为范例）

CubeMX 生成 CMake 项目后，还需要补充以下配置才能与工作区无缝集成。以下按顺序操作：

### 6.1 配置清单

| # | 配置项 | 涉及文件 | 参考命令/内容 |
|---|---|---|---|
| 1 | 🔥 J-Link 烧录脚本 | `flash.jlink.in` + `flash.jlink` | 见下方模板 |
| 2 | 🔧 CMakeLists.txt flash 目标 | `CMakeLists.txt` | 追加 J-Link flash target |
| 3 | 🧹 .clangd 诊断优化 | `.clangd` | 对齐工作区统一配置 |
| 4 | 📁 项目级 tasks.json | `.vscode/tasks.json` | Build + Flash via JLinkExe |
| 5 | 📋 工作区级 tasks.json | `0_Workspace/.vscode/tasks.json` | Build/Flash/All Nodes 入口 |
| 6 | 🗂️ 注册到 robot.code-workspace | `robot.code-workspace` | 加入 `folders` 数组 |
| 7 | ✅ 验证编译+flash 目标 | 命令行 | `cmake --build build/Debug` |

### 6.2 各步骤说明

#### Step 1: flash.jlink.in + flash.jlink

`flash.jlink.in`（CMake `configure_file` 模板）：
```
r
h
loadfile @CMAKE_PROJECT_NAME@.elf
r
q
```

`flash.jlink`（供直接调用，路径相对项目根目录）：
```
r
h
loadfile build/Debug/<项目名>.elf
r
q
```

> **规则**：`flash.jlink` 中的 .elf 路径是**相对项目根目录**的，因此是 `build/Debug/xxx.elf`，而不是绝对路径。

#### Step 2: CMakeLists.txt

在文件末尾（`target_link_libraries(...)` 之后）追加：

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

> **换芯片**：若用 F4 系列，将 `STM32F103C8` 改为对应型号（如 `STM32F407VG`）。

#### Step 3: .clangd

CubeMX 生成的默认 `.clangd` 只有 `CompilationDatabase: build/Debug`，需要补充诊断优化以消除 clangd 误报：

```yaml
CompileFlags:
  Add:
    - '-ferror-limit=0'
    - '-Wno-implicit-int'
  CompilationDatabase: build/Debug
Diagnostics:
  Suppress:
    - unused-includes
    - unknown_typename
    - unknown_typename_suggest
    - typename_requires_specqual
```

#### Step 4: 项目级 .vscode/tasks.json

新建 `.vscode/tasks.json`，提供 Build + Flash 两个任务，Flash 自动依赖 Build：

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

#### Step 5: 工作区级 0_Workspace/.vscode/tasks.json

在 `tasks[]` 数组中添加三项：

```json
{
  "label": "Build: <项目名>",
  "type": "shell",
  "command": "cmake --build ${workspaceFolder}/../<项目路径>/build/Debug",
  "group": "build",
  "problemMatcher": []
},
{
  "label": "Flash: <项目名>",
  "type": "shell",
  "command": "cmake --build ${workspaceFolder}/../<项目路径>/build/Debug --target flash",
  "group": "build",
  "dependsOn": ["Build: <项目名>"],
  "dependsOrder": "parallel",
  "problemMatcher": []
},
```

并更新 `Flash: All Nodes` 任务的 `command` 链和 `dependsOn` 数组。

#### Step 6: robot.code-workspace

在 `folders` 数组中追加新条目：

```json
{
  "name": "<显示名>",
  "path": "<相对 0_Workspace 的路径>"
}
```

> **注意**：`path` 是**相对 `0_Workspace/` 目录**的路径。例如 `3_SteeringArm` 就在工作区根 `../3_SteeringArm`。

#### Step 7: 验证

```bash
cd <项目目录>
cmake --build build/Debug                    # 编译
cmake --build build/Debug --target flash      # 编译+烧录（需连接 J-Link）
```

### 6.3 3_SteeringArm 配置一览

| 配置项 | 值 |
|---|---|
| 项目名 | `3_SteeringArm_t1` |
| MCU | STM32F103C8 |
| 链接脚本 | `STM32F103XX_FLASH.ld` |
| CMOS 生成器 | Ninja |
| 工具链 | `cube-cmake`（STM32CubeMX 封装的 CMake） |
| J-Link 速度 | 4000 kHz |
| clangd | 已启用编译数据库 + 诊断抑制 |
| 烧录方式 | `cmake --target flash` 或直接 `JLinkExe flash.jlink` |
| 工作区 Build 任务 | `Build: SteeringArm` |
| 工作区 Flash 任务 | `Flash: SteeringArm` |

### 参考信息

- **MCU**: STM32F103C8 (Blue Pill)
- **调试器**: SEGGER J-Link（SWD 接口）
- **J-Link 路径**: `/opt/SEGGER/JLink/JLinkExe`
- **编译工具链**: `arm-none-eabi-gcc`
- **CMake Generator**: Ninja
- **烧录速度**: 4000 kHz（部分传感器项目使用 2000 kHz）

### 原有文件结构（未改动）

以下项目不在工作区中或非 CMake 项目，维持原状：

- `6_AK09911C/` — 仅有 .ioc 文件，无 CMakeLists.txt
- `6_NRF24L01/6_Nrf24l01Cmake_test/` — 不在工作区中
- `5_Tec_USART/5_UartToCan_cmake/` — 不在工作区中
- `3_MCLM_t3/` — 不在工作区中
- 所有 `MDK-ARM/` 下的 Keil 项目（uvprojx）
