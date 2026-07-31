#!/usr/bin/env python3
"""
串口通信质量测试 — 接收端
===========================
配合 Windows 串口助手，测试无线串口丢包率和延迟。

用法:
  1. Windows 串口助手: 选 hex发送, 持续发 AA 01 23 02 11 00
  2. Linux 运行:
       python3 serial_quality.py

  统计: 收包率、速率、间隔抖动
"""

import sys, time, argparse, glob, os, threading
import serial

def list_serial_ports():
    ports = []
    for p in sorted(glob.glob('/dev/ttyACM*')): ports.append(p)
    for p in sorted(glob.glob('/dev/ttyUSB*')): ports.append(p)
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        real = os.path.realpath(p)
        if real not in ports: ports.append(p)
    return ports

def select_port():
    ports = list_serial_ports()
    if not ports:
        print("[ERR] 未找到串口设备"); sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        if '/by-id/' in p:
            print(f"  [{i}] {p} → {os.path.realpath(p)}")
        else:
            print(f"  [{i}] {p}")
    while True:
        try:
            idx = int(input(f"选择 [0-{len(ports)-1}]: "))
            if 0 <= idx < len(ports): return ports[idx]
        except: pass

parser = argparse.ArgumentParser()
parser.add_argument('--port', default=None)
parser.add_argument('--baud', type=int, default=115200)
parser.add_argument('--duration', type=int, default=30, help='测试秒数')
args = parser.parse_args()
port = args.port if args.port else select_port()

# ── 统计 ──
total_bytes = 0
pkt_count = 0
pkt_times = []        # 每包到达时间
running = True

ser = serial.Serial(port, args.baud, timeout=0.01)
print(f"[OK] {port} @ {args.baud}")
print(f"[*] 测试 {args.duration}s，等待数据...")
print()

# 统计显示线程
def reporter():
    global running
    last_cnt = 0
    while running:
        time.sleep(1)
        now_cnt = pkt_count
        delta = now_cnt - last_cnt
        last_cnt = now_cnt

        if len(pkt_times) >= 2:
            intervals = [pkt_times[i] - pkt_times[i-1] for i in range(1, len(pkt_times))]
            avg_int = sum(intervals[-100:]) / len(intervals[-100:]) * 1000 if intervals else 0
            jitter = max(intervals[-100:]) - min(intervals[-100:]) if intervals else 0
        else:
            avg_int = 0
            jitter = 0

        elapsed = time.time() - start_t
        rate = total_bytes / elapsed if elapsed > 0 else 0
        sys.stdout.write(f"\r  [{int(elapsed):3d}s] 收包={now_cnt:6d}  {delta:4d}pk/s  "
                         f"间隔={avg_int:5.1f}ms 抖动={jitter*1000:5.1f}ms  速率={rate/1024:.1f}KB/s  ")
        sys.stdout.flush()

start_t = time.time()
t = threading.Thread(target=reporter, daemon=True)
t.start()

try:
    buf = b""
    while time.time() - start_t < args.duration:
        chunk = ser.read(128)
        if not chunk:
            continue

        total_bytes += len(chunk)
        buf += chunk
        now = time.time()

        # 按 \n 拆包（Windows串口助手每帧以\r\n结尾）
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.strip()
            if line:
                pkt_count += 1
                pkt_times.append(now)
                # 只保留最近 1000 个时间戳
                if len(pkt_times) > 1000:
                    pkt_times = pkt_times[-500:]

except KeyboardInterrupt:
    pass
finally:
    running = False
    time.sleep(0.2)
    ser.close()

elapsed = time.time() - start_t
rate = total_bytes / elapsed if elapsed > 0 else 0
print("\n\n" + "=" * 60)
print("  测试结果")
print("=" * 60)
print(f"  测试时间:   {elapsed:.1f}s")
print(f"  接收字节:   {total_bytes}")
print(f"  接收包数:   {pkt_count}")
print(f"  平均速率:   {total_bytes/elapsed/1024:.2f} KB/s" if elapsed > 0 else "N/A")
print(f"  平均收包:   {pkt_count/elapsed:.1f} pk/s" if elapsed > 0 else "N/A")

if len(pkt_times) >= 2:
    intervals = [pkt_times[i] - pkt_times[i-1] for i in range(1, len(pkt_times))]
    avg_int = sum(intervals) / len(intervals) * 1000
    min_int = min(intervals) * 1000
    max_int = max(intervals) * 1000
    print(f"  包间隔平均: {avg_int:.2f} ms")
    print(f"  包间隔最小: {min_int:.2f} ms")
    print(f"  包间隔最大: {max_int:.2f} ms")
    print(f"  抖动(峰峰): {(max_int-min_int):.2f} ms")

loss_pct = 0
print(f"  ─────────────────────────────")
if avg_int > 0 and elapsed > 0:
    expected = elapsed * 1000 / avg_int
    loss = max(0, expected - pkt_count)
    loss_pct = loss / expected * 100 if expected > 0 else 0
    print(f"  估算丢包率: {loss_pct:.2f}%")

if loss_pct < 1 and avg_int < 100:
    print("  ✅ 通信质量良好")
elif loss_pct < 5:
    print("  ⚠️ 通信质量一般")
else:
    print("  ❌ 通信质量差")
print("=" * 60)
