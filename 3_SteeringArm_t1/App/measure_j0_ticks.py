#!/usr/bin/env python3
"""
测 J0 DC 电机编码器 ticks/圈
=============================
用法: python3 App/measure_j0_ticks.py

操作:
  1. 在第三关节上做个标记（转一圈回来）
  2. 按回车电机开始低速转
  3. 转一圈回到标记时按 Ctrl+C
  4. 看 ticks 变化量

协议:
  状态 0x330 → bytes[2-3] = 编码器原始值 (uint16 LE)
  命令 0x130 → [0x11, 0x00, speed_L, speed_H, 0,0,0,0]
"""
import sys, time, can

STATUS_ID = 0x330
CMD_ID    = 0x130

bus = can.interface.Bus('can0', interface='socketcan')
ticks_list = []

def recv_ticks(timeout=0.2):
    """收一帧状态，返回 ticks 或 None"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = bus.recv(timeout=0.05)
        if msg and msg.arbitration_id == STATUS_ID and len(msg.data) >= 4:
            return msg.data[2] | (msg.data[3] << 8)
    return None

def send_speed(speed):
    data = bytes([0x11, 0x00, speed & 0xFF, (speed >> 8) & 0xFF, 0, 0, 0, 0])
    bus.send(can.Message(arbitration_id=CMD_ID, data=data))

print("=" * 60)
print("  J0 编码器 ticks/圈 测量")
print("=" * 60)
print()
print("准备:")
print("  1. 在第三关节壳体上做个标记（如贴纸）")
print("  2. 脚本会让电机低速正转 (speed=30)")
print("  3. 关节转一圈回到标记位置时，按 Ctrl+C")
print()
input("按回车开始...")

# 先收几帧看是否连通
time.sleep(0.2)
init_ticks = recv_ticks()
if init_ticks is None:
    print("❌ 没收到 0x330 状态帧，检查 CAN 通信")
    bus.shutdown()
    sys.exit(1)

print(f"\n初始 ticks = {init_ticks}")
print("\n发 speed=30 正转... 观察关节，转一圈按 Ctrl+C\n")
print("实时 ticks:")
print("-" * 40)

send_speed(30)

last_t = time.time()
try:
    while True:
        ticks = recv_ticks()
        if ticks is not None:
            now = time.time()
            if now - last_t >= 0.2:
                total = ticks - init_ticks
                if total > 32767: total -= 65536
                if total < -32767: total += 65536
                print(f"  ticks={ticks:5d}  total={total:+5d}")
                last_t = now
except KeyboardInterrupt:
    print("\n" + "-" * 40)
    send_speed(0)
    time.sleep(0.2)

    stop_ticks = recv_ticks()
    if stop_ticks is None:
        stop_ticks = init_ticks  # fallback

    total = stop_ticks - init_ticks
    if total > 32767: total -= 65536
    if total < -32767: total += 65536

    print(f"\n结果:")
    print(f"  初始: {init_ticks}")
    print(f"  最终: {stop_ticks}")
    print(f"  变化: {abs(total)} ticks")
    print(f"  方向: {'正转' if total > 0 else '反转'}")
    if abs(total) > 0:
        print(f"\n  → J0 每圈 ≈ {abs(total)} ticks")
        print(f"  → 每度 ≈ {abs(total)/360:.2f} ticks")
        print(f"\n  当前代码设定值: {48} ticks/圈 (12×4)")
        print(f"  {'✅ 一致' if abs(total) == 48 else f'⚠️ 不一致，需更新 J0_ENCODER_TICK_PER_REV'}")
finally:
    send_speed(0)
    bus.shutdown()
