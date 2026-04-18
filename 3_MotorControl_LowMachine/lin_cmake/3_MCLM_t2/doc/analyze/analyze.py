import pandas as pd
import matplotlib.pyplot as plt

# === 1. 读取 CSV ===
import os
_HERE = os.path.dirname(os.path.abspath(__file__))          # doc/analyze/
file = os.path.join(_HERE, "..", "csv", "vofa260418_1231.csv")  # doc/csv/
df = pd.read_csv(file, header=None,
                 names=["SysMs", "M1_HW", "M1_dTick", "M1_Abs", "M2_HW", "M2_dTick", "M2_Abs"])

# === 2. 转换为数值 ===
for col in df.columns:
    df[col] = pd.to_numeric(df[col], errors='coerce')

df = df.dropna()

# === 3. 截取有效段 ===
df = df.iloc[8800:11300]   # ⚠️ 根据实际数据调整

# === 4. 时间轴（SysMs 已是 ms，转 s）===
time = (df["SysMs"] - df["SysMs"].iloc[0]) / 1000.0   # s

# === 5. dt 计算（用于速度反推）===
dt = df["SysMs"].diff() / 1000.0   # ms → s
dt = dt.where(dt > 0)              # 去掉异常（首行/回绕）

# === 6. 由 dTick 反推速度（counts/s）===
# dTick 已是本周期增量（int16，有符号），无需处理溢出
df["M1_speed"] = df["M1_dTick"] / dt
df["M2_speed"] = df["M2_dTick"] / dt

# === 7. 平滑 ===
df["M1_speed_smooth"] = df["M1_speed"].rolling(20).mean()
df["M2_speed_smooth"] = df["M2_speed"].rolling(20).mean()

# ── 图1：M1 & M2 速度对比 ──────────────────────────────────────
plt.figure()
plt.plot(time, df["M1_speed_smooth"], label="M1 Speed (counts/s)")
plt.plot(time, df["M2_speed_smooth"], label="M2 Speed (counts/s)")
plt.legend()
plt.title("M1 & M2 Speed vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Speed (counts/s)")
plt.grid()

# ── 图2：M1 & M2 累积绝对计数 ────────────────────────────────────
plt.figure()
plt.plot(time, df["M1_Abs"], label="M1 Abs Count")
plt.plot(time, df["M2_Abs"], label="M2 Abs Count")
plt.legend()
plt.title("M1 & M2 Absolute Encoder Count vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Encoder Count (counts)")
plt.grid()

# ── 图3：M1 & M2 TIM 寄存器原始值 ───────────────────────────────
plt.figure()
plt.plot(time, df["M1_HW"], label="M1 TIM2 HW")
plt.plot(time, df["M2_HW"], label="M2 TIM3 HW")
plt.legend()
plt.title("M1 & M2 TIM Register (HW) vs Time")
plt.xlabel("Time (s)")
plt.ylabel("TIM Counter (0~65535)")
plt.grid()

# ── 图4：M1 & M2 本周期增量 dTick ────────────────────────────────
plt.figure()
plt.plot(time, df["M1_dTick"], label="M1 dTick")
plt.plot(time, df["M2_dTick"], label="M2 dTick")
plt.legend()
plt.title("M1 & M2 dTick per Period vs Time")
plt.xlabel("Time (s)")
plt.ylabel("dTick (counts/period)")
plt.grid()

# === 统一显示 ===
plt.show()
