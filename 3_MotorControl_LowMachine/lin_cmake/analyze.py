import pandas as pd
import matplotlib.pyplot as plt

# === 1. 读取 CSV ===
file = "/home/lin/ProjectRequirement/MCU/Lin_STM32/STM32_F103C8T6/STM32_Now/3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2/doc/vofa260417_1933.csv"
df = pd.read_csv(file)

# === 2. 重命名列 ===
df = df.rename(columns={
    "I0": "tick",
    "I1": "encoder",
    "I2": "speed",
    "I3": "pwm"
})

# === 3. 转换为数值 ===
for col in ["tick", "encoder", "speed", "pwm"]:
    df[col] = pd.to_numeric(df[col], errors='coerce')

df = df.dropna()

# === 4. 截取“中间有效段”（关键！）===
df = df.iloc[0:2500]   # ⚠️ 不要用前2000

# === 5. 编码器差分（处理溢出）===
diff = df["encoder"].diff()

diff[diff > 30000] -= 65536
diff[diff < -30000] += 65536

df["delta_encoder"] = diff

# === 6. 时间计算 ===
df["dt"] = df["tick"].diff() / 1000.0   # 如果tick是us → 改 /1_000_000

# 去掉异常
df = df[df["dt"] > 0]

# === 7. 编码器反推速度 ===
df["speed_calc"] = df["delta_encoder"] / df["dt"]

# === 8. 平滑（非常关键）===
df["speed_calc_smooth"] = df["speed_calc"].rolling(20).mean()

# === 9. 构造真实时间轴 ===
time = (df["tick"] - df["tick"].iloc[0]) / 1000.0

# === 10. 速度对比 ===
plt.figure()
plt.plot(time, df["speed"], label="MCU speed")
plt.plot(time, df["speed_calc_smooth"], label="Encoder speed")
plt.legend()
plt.title("Speed Compare")
plt.xlabel("Time (s)")
plt.ylabel("Speed")
plt.grid()

# === 11. PWM vs 速度（排序后更清晰）===
plt.figure()
df_sorted = df.sort_values("pwm")
plt.plot(df_sorted["pwm"], df_sorted["speed"], '.')
plt.title("PWM vs Speed")
plt.xlabel("PWM")
plt.ylabel("Speed")
plt.grid()

# === 12. PWM & Speed 随时间变化（强烈推荐🔥）===
plt.figure()
plt.plot(time, df["pwm"], label="PWM")
plt.plot(time, df["speed"], label="Speed")
plt.legend()
plt.title("PWM & Speed vs Time")
plt.xlabel("Time (s)")
plt.grid()

# === 统一显示 ===
plt.show()