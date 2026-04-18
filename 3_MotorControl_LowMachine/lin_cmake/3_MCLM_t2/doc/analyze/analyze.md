# analyze.py 说明

## 运行方式

```bash
# 任意目录下均可，路径由脚本自身定位
python3 doc/analyze/analyze.py
```

脚本通过 `__file__` 定位自身所在目录（`doc/analyze/`），再向上一级找 `doc/csv/`，
**与当前工作目录无关**，不会出现路径错误。

---

## CSV 格式

由 `logger_task.c` 输出，共 7 列，**无表头**，采样周期 10 ms：

| 列名 | 类型 | 说明 |
|------|------|------|
| `SysMs` | uint32 | FreeRTOS 系统时间（ms） |
| `M1_HW` | uint16 | M1 — TIM2 寄存器当前值（0 ~ 65535） |
| `M1_dTick` | int16 | M1 — 本周期增量脉冲数（有符号） |
| `M1_Abs` | int32 | M1 — 软件累积绝对计数（上电后持续累加） |
| `M2_HW` | uint16 | M2 — TIM3 寄存器当前值（0 ~ 65535） |
| `M2_dTick` | int16 | M2 — 本周期增量脉冲数（有符号） |
| `M2_Abs` | int32 | M2 — 软件累积绝对计数（上电后持续累加） |

> Logger_Task 由 Encoder_Task 每 10 ms 触发一次（`osThreadFlagsWait(0x01)`）。

---

## 数据处理流程

```
CSV (7列, 无表头)
    │
    ├─ 1. 读取：header=None，手动指定列名
    │
    ├─ 2. 转数值 + dropna（过滤非数值行）
    │
    ├─ 3. 截取有效段  df.iloc[8800:11300]   ← 根据实际数据调整
    │
    ├─ 4. 时间轴  time = (SysMs - SysMs[0]) / 1000.0   [s]
    │
    ├─ 5. dt = diff(SysMs) / 1000.0   [s]，过滤 dt ≤ 0
    │
    ├─ 6. 速度反推
    │       M1_speed = M1_dTick / dt   [counts/s]
    │       M2_speed = M2_dTick / dt
    │       （dTick 已是有符号增量，MCU 侧已处理溢出，Python 侧无需额外处理）
    │
    └─ 7. 平滑  rolling(20).mean()  ≈ 200 ms 窗口
```

### 关键设计说明

| 项 | 说明 |
|----|------|
| **无溢出修正** | 旧版需对原始 HW 值做差分并修正 ±30000 跳变；新版 `dTick` 由 MCU 侧处理，Python 侧直接用 |
| **时间单位** | `SysMs` 本身为 ms，除以 1000 得秒，无需 `/ 1_000_000` |
| **平滑窗口** | `rolling(20)` 对应约 200 ms，可按需调整 |
| **有效段截取** | `df.iloc[8800:11300]` 跳过启动瞬态，需根据当前 CSV 行数手动核对 |

---

## 输出图表

| 图 | 标题 | 内容 |
|----|------|------|
| 1 | M1 & M2 Speed vs Time | M1/M2 平滑速度（counts/s）随时间变化 |
| 2 | M1 & M2 Absolute Encoder Count vs Time | M1/M2 累积绝对计数随时间变化 |
| 3 | M1 & M2 TIM Register (HW) vs Time | TIM2/TIM3 寄存器原始值（锯齿波，0~65535） |
| 4 | M1 & M2 dTick per Period vs Time | 每 10 ms 周期的增量脉冲数 |

---

## 注意事项

- 若 CSV 第一行为表头字符串，将 `header=None, names=[...]` 替换为直接 `pd.read_csv(file)` 即可。
- 切换 CSV 文件只需修改脚本第 7 行的文件名：
  ```python
  file = os.path.join(_HERE, "..", "csv", "新文件名.csv")
  ```
- 速度单位为 **counts/s**，换算为 rpm 需结合编码器线数和减速比：
  ```
  rpm = speed_counts_s / (PPR × 减速比) × 60
  ```
