---
name: project-rename-map-error
description: STM32CubeIDE扩展缓存key=文件夹名而非项目名，改名后map文件不匹配的修复
metadata: 
  node_type: memory
  type: reference
  originSessionId: c60f0f6a-c3d0-4ac7-b29c-a62d50a91f1b
---

CubeMX 中修改 `.ioc` 项目名后（如 `3_SteeringArm` → `3_SteeringArm_t1`），VS Code 的 STM32CubeIDE Build Analyzer 扩展报 "Map file data is not available"，因为其缓存 key 是**工作区文件夹名**（`3_SteeringArm`），而非 `.ioc` 中的项目名（`3_SteeringArm_t1`）。

**Why:** STM32 扩展用工作区文件夹路径做缓存 key，去 build 目录找 `<文件夹名>.map`，但实际编译产物是 `<项目名>.map`。

**How to avoid:** 确保文件夹名 = CMAKE_PROJECT_NAME（即 .ioc 项目名），改名时两者同步改。

**How to fix if inconsistent:** 删除 `~/.config/Code/User/workspaceStorage/<hash>/state.vscdb`，重载 VS Code 窗口。扩展会重新扫描并生成正确的缓存。

[[project-structure]]
