#!/usr/bin/env python3
"""
R1 键盘遥控 — Windows 直接读按键状态
=======================================
用 ctypes.GetAsyncKeyState 读物理按键，不依赖控制台缓冲区。

安装: pip install pyserial
运行: python keyboard_control.py

按键:
  W 前进  S 后退  A 左移  D 右移  Q 左转  E 右转
  Space 急停  ESC 退出
"""

import sys, serial, glob, time, ctypes

WIN = sys.platform.startswith('win')
SEND_INTERVAL = 0.2
MAX_VX = MAX_VY = MAX_OM = 0.3

# ── 用 ctypes 读取按键状态 ──
if WIN:
    user32 = ctypes.windll.user32
    VK = {'W': 0x57, 'S': 0x53, 'A': 0x41, 'D': 0x44, 'Q': 0x51, 'E': 0x45, 'SPACE': 0x20, 'ESC': 0x1B}
    def key_down(vk):
        return bool(user32.GetAsyncKeyState(vk) & 0x8000)

def list_ports():
    ports = []
    if WIN:
        try:
            import serial.tools.list_ports
            return [p.device for p in serial.tools.list_ports.comports()]
        except:
            return ["COM1"]
    else:
        for p in sorted(glob.glob('/dev/ttyACM*')): ports.append(p)
        for p in sorted(glob.glob('/dev/ttyUSB*')): ports.append(p)
    return ports

def select_port():
    ports = list_ports()
    if not ports:
        print("[ERR] 未找到串口"); sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports): print(f"  [{i}] {p}")
    while True:
        try:
            idx = int(input(f"选择 [0-{len(ports)-1}]: ")); return ports[idx]
        except: pass

port = select_port()
baud = input("波特率 (默认115200): ").strip()
baud = int(baud) if baud else 115200

try:
    ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1)
except Exception as e:
    print(f"[ERR] {e}"); sys.exit(1)

print(f"\n[OK] {port} @ {baud}")
print("W前进 S后退 A左移 D右移 Q左转 E右转")
print("Space急停 ESC退出  按住即动松手停")
print()

running = True
last_cmd = ""

try:
    while running:
        if WIN:
            # 读取物理按键状态（GetAsyncKeyState）
            w = key_down(VK['W'])
            s = key_down(VK['S'])
            a = key_down(VK['A'])
            d = key_down(VK['D'])
            q = key_down(VK['Q'])
            e = key_down(VK['E'])
            space = key_down(VK['SPACE'])
            esc = key_down(VK['ESC'])
        else:
            # Linux 暂不实现
            w = s = a = d = q = e = space = esc = False

        # 急停
        if space:
            ser.write(b"0 0 0\n")
            w = s = a = d = q = e = False
            print("\r  ⛔ 急停                         ", end='')
            sys.stdout.flush()
            time.sleep(0.3)
            continue

        # 退出
        if esc:
            running = False
            break

        # 互斥
        if w: s = False
        if s: w = False
        if a: d = False
        if d: a = False
        if q: e = False
        if e: q = False

        # 计算指令
        vx = (1 if w else 0) - (1 if s else 0)
        vy = (1 if a else 0) - (1 if d else 0)
        om = (1 if e else 0) - (1 if q else 0)  # E=右转(+), Q=左转(-)
        vx *= MAX_VX; vy *= MAX_VY; om *= MAX_OM
        cmd = f"{vx:.2f} {vy:.2f} {om:.2f}\n"

        try:
            ser.write(cmd.encode())
        except Exception as ex:
            print(f"\r[ERR] 串口写入: {ex}", end='')
            continue

        if vx != 0 or vy != 0 or om != 0:
            s = ('W' if w else '-') + ('S' if s else '-') + ('A' if a else '-') + ('D' if d else '-') + ('Q' if q else '-') + ('E' if e else '-')
            print(f"\r  [{s}]  vx={vx:+.2f} vy={vy:+.2f} ω={om:+.2f}  ", end='')
        else:
            print(f"\r  ● 停止                          ", end='')
        sys.stdout.flush()

        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    pass
finally:
    ser.write(b"0 0 0\n"); ser.close()
    print("\n退出")
