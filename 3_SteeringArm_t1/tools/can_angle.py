#!/usr/bin/env python3
"""角度/速度 ↔ CAN十六进制 快速转换工具
   角度: int16 小端, 0.1°精度, -1500~+1500 对应 -150.0°~+150.0°
   速度: int16 小端, 0.1°/s, 如 1800 = 180.0°/s

   用法: python3 can_angle.py
"""
import struct

def int16_le(val):
    return struct.pack('<h', val).hex().upper()

def uint16_le(val):
    return struct.pack('<H', val).hex().upper()

def angle_to_hex(angle_deg):
    val = int(round(angle_deg * 10))
    return int16_le(val)

def speed_to_hex(speed_dps):
    val = int(round(speed_dps * 10))
    return int16_le(val)

def joint_to_id(name):
    m = {'j1': 1, 'j2': 2, 'g': 3, 'gripper': 3}
    return m.get(name.lower(), None)

def fmt_can_raw(id_hex, data_hex):
    """cansend 格式: 130#1101840300000000"""
    return f"{id_hex}#{data_hex}"

def fmt_can_vis(id_hex, data_hex_parts):
    """可读格式: 130#11 01 84 03 00 00 00 00"""
    return f"{id_hex}#{' '.join(data_hex_parts)}"

def make_data(prefix_hex):
    """将 hex 前缀补齐到 16 hex chars (8 bytes)"""
    return prefix_hex.ljust(16, '0')[:16]


def main():
    print("=" * 60)
    print("CAN 机械臂命令生成器  (q 退出)")
    print("=" * 60)
    print()
    print("命令:")
    print("  <角度值>           → hex, 如: 90")
    print("  <lo> <hi>          → 角度, 如: 84 03")
    print("  j1 <角度>          → J1 绝对, 如: j1 90")
    print("  j2 <角度>          → J2 绝对, 如: j2 -45")
    print("  g <脉宽>           → 夹爪绝对, 如: g 3000")
    print("  inc j1 <角度>      → J1 增量, 如: inc j1 10")
    print("  inc j2 <角度>      → J2 增量")
    print("  spd <速度>         → 统一速度, 如: spd 180")
    print("  home               → 回中 (0x430 01)")
    print("  lock               → 加锁 (0x430 03)")
    print("  unlock             → 解锁 (0x430 04)")
    print()

    while True:
        inp = input("> ").strip()
        if inp.lower() in ('q', 'quit', 'exit'):
            break

        parts = inp.split()
        if not parts:
            continue

        # === 回中 ===
        if parts[0] == 'home':
            print(f"  COPY: cansend can0 {fmt_can_raw('430', '0100000000000000')}")
            print(f"  READ: {fmt_can_vis('430', ['01', '00', '00', '00', '00', '00', '00', '00'])}")
            continue

        # === 加锁/解锁 ===
        if parts[0] == 'lock':
            print(f"  COPY: cansend can0 {fmt_can_raw('430', '0300000000000000')}")
            print(f"  READ: {fmt_can_vis('430', ['03', '00', '00', '00', '00', '00', '00', '00'])}")
            continue
        if parts[0] == 'unlock':
            print(f"  COPY: cansend can0 {fmt_can_raw('430', '0400000000000000')}")
            print(f"  READ: {fmt_can_vis('430', ['04', '00', '00', '00', '00', '00', '00', '00'])}")
            continue

        # === 速度命令: spd <速度> ===
        if parts[0] == 'spd' and len(parts) == 2:
            try:
                h = speed_to_hex(float(parts[1]))
                raw = f"430#02{h}00000000"
                vis = f"430#02 {h[0:2]} {h[2:4]} 00 00 00 00 00 00"
                print(f"  → Speed = {parts[1]}°/s (J1+J2 统一)")
                print(f"  COPY: cansend can0 {raw}")
                print(f"  READ: {vis}")
                continue
            except ValueError:
                pass

        # === 增量命令: inc <关节> <角度> ===
        if parts[0] == 'inc' and len(parts) == 3:
            joint_id = joint_to_id(parts[1])
            if joint_id is None:
                print(f"  未知关节: {parts[1]}, 可用: j1 j2 g")
                continue
            try:
                h = angle_to_hex(float(parts[2]))
                raw = f"130#{make_data('21' + f'{joint_id:02X}' + h)}"
                vis = f"130#21 {joint_id:02X} {h[0:2]} {h[2:4]} 00 00 00 00"
                joint_name = {1: 'J1', 2: 'J2', 3: 'Gripper'}[joint_id]
                print(f"  → {joint_name} += {parts[2]}° (增量)")
                print(f"  COPY: cansend can0 {raw}")
                print(f"  READ: {vis}")
                continue
            except ValueError:
                pass

        # === 绝对命令: <关节> <角度> ===
        if parts[0].lower() in ('j1', 'j2', 'g', 'gripper') and len(parts) == 2:
            joint_id = joint_to_id(parts[0])
            try:
                # 夹爪用 uint16，关节用 int16 角度
                if joint_id == 3:
                    val = int(parts[1])
                    h = uint16_le(val)
                    raw = f"130#{make_data('11' + f'{joint_id:02X}' + h)}"
                    vis = f"130#11 {joint_id:02X} {h[0:2]} {h[2:4]} 00 00 00 00"
                    print(f"  → Gripper = {val} (CCR)")
                else:
                    h = angle_to_hex(float(parts[1]))
                    raw = f"130#{make_data('11' + f'{joint_id:02X}' + h)}"
                    vis = f"130#11 {joint_id:02X} {h[0:2]} {h[2:4]} 00 00 00 00"
                    joint_name = {1: 'J1', 2: 'J2'}[joint_id]
                    print(f"  → {joint_name} = {parts[1]}° (绝对)")
                print(f"  COPY: cansend can0 {raw}")
                print(f"  READ: {vis}")
                continue
            except ValueError:
                pass

        # === 两字节 → 角度 ===
        if len(parts) == 2 and all(len(p) <= 2 for p in parts):
            try:
                lo = int(parts[0], 16)
                hi = int(parts[1], 16)
                val = (hi << 8) | lo
                if val >= 0x8000:
                    val -= 0x10000
                print(f"  → {val / 10.0}°")
                continue
            except ValueError:
                pass

        # === 纯数字 → hex ===
        try:
            v = float(inp)
            h = angle_to_hex(v)
            print(f"  → {h}  (编码={int(round(v*10))})")
        except ValueError:
            print("  无效输入。输入 q 退出。")
            print("  命令: j1/j2/g <角度> | inc j1/j2 <角度> | spd <速度> | home")


if __name__ == "__main__":
    main()
