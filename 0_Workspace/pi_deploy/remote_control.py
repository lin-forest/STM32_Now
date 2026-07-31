#!/usr/bin/env python3
"""
遥控端 — 从本机串口发送底盘控制指令
========================================
通过无线串口模块向树莓派发送 vx vy omega 指令。

接线: PC USB → USB转TTL → E34模块 → 无线 → E34模块 → 树莓派

用法:
  python3 remote_control.py                      # 交互选择串口
  python3 remote_control.py --port /dev/ttyUSB0  # 指定串口

交互模式命令:
  0.5 0 0        → 前进 0.5 m/s
  0 0.2 0        → 左移 0.2 m/s
  0 0 0.3        → 左转 0.3 rad/s
  stop           → 停止
  q              → 退出
"""

import sys, time, argparse, glob, os, threading
import serial

def list_ports():
    ports = []
    for p in sorted(glob.glob('/dev/ttyACM*')): ports.append(p)
    for p in sorted(glob.glob('/dev/ttyUSB*')): ports.append(p)
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        real = os.path.realpath(p)
        if real not in ports: ports.append(p)
    return ports

def select_port():
    ports = list_ports()
    if not ports:
        print("[ERR] 未找到串口设备"); sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        if '/by-id/' in p: print(f"  [{i}] {p} → {os.path.realpath(p)}")
        else: print(f"  [{i}] {p}")
    while True:
        try:
            idx = int(input(f"选择 [0-{len(ports)-1}]: "))
            if 0 <= idx < len(ports): return ports[idx]
        except: pass

parser = argparse.ArgumentParser()
parser.add_argument('--port', default=None)
parser.add_argument('--baud', type=int, default=115200)
args = parser.parse_args()
port = args.port if args.port else select_port()

ser = serial.Serial(port, args.baud, timeout=1)
print(f"[OK] 串口 {port} @ {args.baud}")
print("-" * 40)
print("  输入 vx vy omega 发送指令")
print("  例: 0.3 0 0   → 前进")
print("       0 0.2 0   → 左移")
print("       0 0 0.3   → 自转")
print("       stop      → 停止")
print("       q         → 退出")
print("-" * 40)

try:
    while True:
        line = input("> ").strip()
        if not line: continue
        if line == 'q': break
        if not line.endswith('\n'):
            line += '\n'
        ser.write(line.encode())
        print(f"  → {port}: {repr(line)}")
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("退出")
