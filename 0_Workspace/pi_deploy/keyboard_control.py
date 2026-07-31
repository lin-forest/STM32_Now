#!/usr/bin/env python3
"""
R1 键盘遥控 — 底盘控制 + 单轮调校
=====================================
底盘模式: W/A/S/D/Q/E 控制车体
轮控模式: 按 [ 进入，数字选轮，H保持 F释放 +/-微调 ]退出

安装: pip install pyserial
运行: python keyboard_control.py
"""

import sys, serial, glob, time

WIN = sys.platform.startswith('win')
SEND_INTERVAL = 0.2
SPEED = {1: 0.15, 2: 0.30, 3: 0.60}
cur_spd = 2

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
    if not ports: print("[ERR] 未找到串口"); sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports): print(f"  [{i}] {p}")
    while True:
        try: idx = int(input(f"选择 [0-{len(ports)-1}]: ")); return ports[idx]
        except: pass

port = select_port()
baud = input("波特率 (默认115200): ").strip()
baud = int(baud) if baud else 115200

try:
    ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1)
except Exception as e:
    print(f"[ERR] {e}"); sys.exit(1)

print(f"\n[OK] {port} @ {baud}")
print("WASDQE=底盘  1/2/3=速度")
print("[ =轮控模式  ]=退出轮控  数字选轮 H保持 F自由 +/-微调")
print()

if WIN:
    import msvcrt

keys = set()
key_last_seen = {}
KEY_HOLD_TIMEOUT = 0.4
running = True
wheel_mode = False     # 轮控模式
sel_unit = 0           # 选中的单元 (0=无)
arm_mode = False       # 臂控模式
_preset_idx = 0        # 预设序号
heartbeat = 0

try:
    while running:
        heartbeat += 1
        now = time.time()
        pressed = set()
        if WIN:
            while msvcrt.kbhit():
                ch = msvcrt.getch()
                if isinstance(ch, bytes):
                    c = ch.decode(errors='ignore')
                else:
                    c = ch
                if c == '\xe0':
                    ch2 = msvcrt.getch()
                    arrow = {b'H': 'w', b'P': 's', b'K': 'a', b'M': 'd'}
                    c = arrow.get(ch2 if isinstance(ch2, bytes) else bytes([ch2]), '')
                pressed.add(c)

        # ── 急停/退出（通用）──
        if ' ' in pressed:
            ser.write(b"0 0 0\n")
            keys.clear()
            print("\r  ⛔ 急停                         ", end='')
            sys.stdout.flush(); time.sleep(0.3); continue
        if '.' in pressed:
            running = False; break

        # ── 轮控模式 ──
        if '[' in pressed:
            wheel_mode = True
            sel_unit = 0
            print("\r  🎯 轮控模式: 数字1-4选轮, H保持 F自由 +/-微调 ]退出  ")
            sys.stdout.flush()
        if wheel_mode:
            for c in pressed:
                if c == ']':
                    wheel_mode = False
                    sel_unit = 0
                    print("\r  ⌨️ 底盘模式                              ")
                elif c in '1234':
                    sel_unit = int(c)
                    print(f"\r  🎯 已选 UNIT{c} (H保持 F自由 +/-微调)      ")
                elif c == 'h' and sel_unit:
                    ser.write(f"wheel {sel_unit} hold\n".encode())
                    print(f"\r  🛞 UNIT{sel_unit} 保持                    ")
                elif c == 'f' and sel_unit:
                    ser.write(f"wheel {sel_unit} free\n".encode())
                    print(f"\r  🛞 UNIT{sel_unit} 释放                    ")
                elif c == '+' and sel_unit:
                    ser.write(f"wheel {sel_unit} +5\n".encode())
                    print(f"\r  🛞 UNIT{sel_unit} +5°                     ")
                elif c == '-' and sel_unit:
                    ser.write(f"wheel {sel_unit} -5\n".encode())
                    print(f"\r  🛞 UNIT{sel_unit} -5°                     ")
            sys.stdout.flush()
            time.sleep(SEND_INTERVAL)
            continue  # 轮控模式不处理底盘

        # ── 臂控模式 ──
        if ';' in pressed:
            arm_mode = True
            arm_inc = 5
            arm_s = 0; arm_e = 0  # 本地跟踪位置
            ser.write(b"arm init\n")   # 初始化+进CSP
            time.sleep(0.5)
            ser.write(b"arm pos\n")
            time.sleep(0.3)
            resp = ser.read(64).decode(errors='ignore').strip()
            if resp.startswith('[POS]'):
                parts = resp.split()
                arm_s = float(parts[1]); arm_e = float(parts[2])
            print(f"\r  🦾 臂控: 肩{arm_s:.0f}°肘{arm_e:.0f}° L锁死 F放松 W/S肩 A/D肘 1~9预设 H归零 P存 t温度 ]退出  ", end='')
            sys.stdout.flush()
        if arm_mode:
            for c in pressed:
                if c == ']':
                    arm_mode = False
                    print("\r  ⌨️ 底盘模式                              ")
                elif c in 'ws':
                    d = 1 if c == 'w' else -1
                    arm_s = max(-148, min(0, arm_s + d * arm_inc))
                    ser.write(f"arm s {arm_s:.0f}\n".encode())
                    print(f"\r  🦾 肩 {arm_s:.0f}°                        ")
                elif c in 'ad':
                    d = 1 if c == 'a' else -1
                    arm_e = max(0, min(130, arm_e + d * arm_inc))
                    ser.write(f"arm e {arm_e:.0f}\n".encode())
                    print(f"\r  🦾 肘 {arm_e:.0f}°                        ")
                elif c in 'WS':  # Shift+W/S = 肩 ±10°
                    d = 1 if c == 'W' else -1
                    arm_s = max(-148, min(0, arm_s + d * 10))
                    ser.write(f"arm s {arm_s:.0f}\n".encode())
                    print(f"\r  🦾 肩 {arm_s:.0f}° [×2]                     ")
                elif c in 'AD':  # Shift+A/D = 肘 ±10°
                    d = 1 if c == 'A' else -1
                    arm_e = max(0, min(130, arm_e + d * 10))
                    ser.write(f"arm e {arm_e:.0f}\n".encode())
                    print(f"\r  🦾 肘 {arm_e:.0f}° [×2]                     ")
                elif c == '8':
                    ser.write(b"arm speed 0.03\n")
                    print("\r  🦾 臂速: 🐢 0.03 rad/s                    ")
                elif c == '9':
                    ser.write(b"arm speed 0.05\n")
                    print("\r  🦾 臂速: 🐇 0.05 rad/s                    ")
                elif c == '0':
                    ser.write(b"arm speed 0.10\n")
                    print("\r  🦾 臂速: 🚀 0.10 rad/s                    ")
                elif c == 'l':
                    ser.write(b"arm lock\n")
                    time.sleep(0.5)
                    ser.write(b"arm pos\n"); time.sleep(0.2)
                    resp = ser.read(64).decode(errors='ignore').strip()
                    if resp.startswith('[POS]'):
                        parts = resp.split()
                        arm_s = float(parts[1]); arm_e = float(parts[2])
                    print(f"\r  🔒 已锁死 @ 肩{arm_s:.0f}° 肘{arm_e:.0f}°                ")
                elif c == 'f':
                    ser.write(b"arm free\n")
                    print("\r  🔓 已放松，可手动掰                       ")
                elif c == 'h':
                    ser.write(b"arm home\n")
                    arm_s, arm_e = -3, 6
                    print(f"\r  🦾 归零 (-3°,6°)                            ", end='')
                elif c == 'z':
                    ser.write(b"arm pump on\n")
                    print("\r  🌀 气泵开启                            ")
                elif c == 'x':
                    ser.write(b"arm pump off\n")
                    print("\r  🌀 气泵关闭                            ")
                elif c == 't':
                    ser.write(b"arm temp\n")
                    time.sleep(0.3)
                    resp = ser.read(64).decode(errors='ignore').strip()
                    if resp:
                        print(f"\r  🌡️ {resp}                    ")
                    else:
                        print("\r  🌡️ 无响应                            ")
                elif c == 'p':
                    _preset_idx = (_preset_idx % 9) + 1
                    ser.write(f"arm preset save {_preset_idx}\n".encode())
                    print(f"\r  🦾 预设 #{_preset_idx} 已保存               ")
                elif c in '123456789':
                    ser.write(f"arm preset {c}\n".encode())
                    time.sleep(0.3)
                    ser.write(b"arm pos\n"); time.sleep(0.2)
                    resp = ser.read(64).decode(errors='ignore').strip()
                    if resp.startswith('[POS]'):
                        parts = resp.split()
                        arm_s = float(parts[1]); arm_e = float(parts[2])
                    print(f"\r  🦾 预设#{c} → 肩{arm_s:.0f}° 肘{arm_e:.0f}°           ")
            sys.stdout.flush()
            time.sleep(SEND_INTERVAL)
            continue

        # ── 底盘模式：更新按键 ──
        for k in 'wsadqe':
            if k in pressed:
                keys.add(k); key_last_seen[k] = now
                if k == 'w': keys.discard('s')
                if k == 's': keys.discard('w')
                if k == 'a': keys.discard('d')
                if k == 'd': keys.discard('a')
                if k == 'q': keys.discard('e')
                if k == 'e': keys.discard('q')
            else:
                if now - key_last_seen.get(k, 0) > KEY_HOLD_TIMEOUT:
                    keys.discard(k)

        # 调速
        if '1' in pressed: cur_spd = 1
        if '2' in pressed: cur_spd = 2
        if '3' in pressed: cur_spd = 3
        max_v = SPEED[cur_spd]

        # 计算发送
        vx = (1 if 'w' in keys else 0) - (1 if 's' in keys else 0)
        vy = (1 if 'a' in keys else 0) - (1 if 'd' in keys else 0)
        om = (1 if 'e' in keys else 0) - (1 if 'q' in keys else 0)
        vx *= max_v; vy *= max_v; om *= max_v
        cmd = f"{vx:.2f} {vy:.2f} {om:.2f}\n"

        try: ser.write(cmd.encode())
        except: continue

        if vx != 0 or vy != 0 or om != 0:
            ks = ''.join(k.upper() if k in keys else '-' for k in 'wsadqe')
            print(f"\r  [{ks}]  v={max_v:.2f}  vx={vx:+.2f} vy={vy:+.2f} ω={om:+.2f}  ", end='')
        else:
            print(f"\r  ● 停止 [速度{cur_spd}]", end='')
        sys.stdout.flush()
        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    pass
finally:
    ser.write(b"0 0 0\n"); ser.close()
    print("\n退出")
