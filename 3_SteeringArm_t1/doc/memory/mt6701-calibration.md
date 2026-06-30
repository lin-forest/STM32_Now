---
name: mt6701-calibration
description: MT6701 磁编码器标定参数（J1/J2 中位偏移、行程范围、回绕处理）
metadata: 
  node_type: memory
  type: reference
  originSessionId: 4844ece2-d4b7-4e4d-ba86-32fadaaf99f7
---

## MT6701 标定参数（3_SteeringArm_t1）

### J1（肩关节舵机）
- **中位偏移 (0°)**：raw = 16300
- **最小收缩端（默认姿态）**：raw = 12041 → -93.6°
- **最大伸出端**：raw = 4048 → +90.8°（经过 16383→0 回绕）
- **回绕处理**：必须（中位距 16383 仅 83 raw）

### J2（肘关节舵机）
- **中位偏移 (0°)**：raw = 12200
- **最小收缩端（默认姿态）**：raw = 7637 → -100.3°
- **最大伸出端**：raw = 16200 → +87.9°
- **回绕处理**：建议加（最大伸出距 16383 仅 183 raw）

### 相关函数
- `MT6701_RawToAngleX10(raw, offset_raw)` — raw → 角度×10，含回绕
- `MT6701_GetAngleX10(cs_port, cs_pin, offset_raw)` — 读 SPI + 转换

### 角度约定
- 0° = 舵机中位（脉宽 1500μs）
- 负角度 = 收缩方向
- 正角度 = 伸出方向
