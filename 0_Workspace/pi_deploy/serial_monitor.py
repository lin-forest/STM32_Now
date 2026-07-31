#!/usr/bin/env python3
"""
串口监视器 — 接收并显示串口数据（原始二进制，逐包显示）
========================================================
用法:
  python3 serial_monitor.py                     # 交互选择串口
  python3 serial_monitor.py --port /dev/ttyACM0 # 指定串口
  python3 serial_monitor.py --baud 115200
"""

import sys, time, argparse, glob, os
import serial

def list_serial_ports():
    ports = []
    for p in sorted(glob.glob('/dev/ttyACM*')):
        ports.append(p)
    for p in sorted(glob.glob('/dev/ttyUSB*')):
        ports.append(p)
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        real = os.path.realpath(p)
        if real not in ports:  # 去重
            ports.append(p)
    return ports

def select_port():
    ports = list_serial_ports()
    if not ports:
        print("[ERR] 未找到任何串口设备")
        sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        if '/by-id/' in p:
            real = os.path.realpath(p)
            print(f"  [{i}] {p}")
            print(f"       → {real}")
        else:
            print(f"  [{i}] {p}")
    while True:
        try:
            idx = int(input(f"请选择串口 [0-{len(ports)-1}]: "))
            if 0 <= idx < len(ports):
                return ports[idx]
        except (ValueError, IndexError):
            pass
        print("  输入无效，请重试")

parser = argparse.ArgumentParser()
parser.add_argument('--port', default=None)
parser.add_argument('--baud', type=int, default=115200)
args = parser.parse_args()

port = args.port if args.port else select_port()

try:
    ser = serial.Serial(port, args.baud, timeout=0.05)
    print(f"[OK] 串口 {port} @ {args.baud} 已打开")
    print(f"[*] 等待接收数据... (Ctrl+C 退出)")
    print("-" * 70)

    total_bytes = 0
    start_time = time.time()

    while True:
        try:
            chunk = ser.read(64)
            if not chunk:
                continue

            total_bytes += len(chunk)
            now = time.time()
            ts = time.strftime('%H:%M:%S', time.localtime(now))
            rate = total_bytes / (now - start_time) if (now - start_time) > 0 else 0

            # 每收到一块数据立即显示
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)

            print(f"[{ts}] 收到{len(chunk):3d}字节 速率{rate/1024:.1f}KB/s")
            print(f"       HEX:  {hex_str}")
            print(f"       TEXT: {ascii_str}")
            print()

        except serial.SerialException as e:
            print(f"[ERR] {e}")
            time.sleep(1)

except KeyboardInterrupt:
    elapsed = time.time() - start_time
    print()
    print("-" * 70)
    print(f"  接收结束: {total_bytes} 字节, {elapsed:.1f}s")
    print(f"  平均速率: {total_bytes/elapsed/1024:.2f} KB/s")
    if 'ser' in dir() and ser:
        ser.close()
except Exception as e:
    print(f"[ERR] {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
