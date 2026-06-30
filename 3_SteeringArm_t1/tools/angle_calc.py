#!/usr/bin/env python3
"""
角度 ↔ CAN 编码 转换器

角度约定：0° = 中位，-150° ~ +150°
CAN 编码：int16 小端, 0.1° 精度

用法:
  python3 angle_calc.py <角度>
  python3 angle_calc.py <hex1> <hex2>
  python3 angle_calc.py --joint <角度>

示例:
  python3 angle_calc.py 90        # 角度→编码: 90° → 84 03
  python3 angle_calc.py 84 03     # 编码→角度: 0x84 0x03 → 90°
  python3 angle_calc.py -45       # 角度→编码: -45° → 3E FE
  python3 angle_calc.py 3E FE     # 编码→角度: 0x3E 0xFE → -45°
"""

import sys
import struct

def angle_to_hex(deg):
    """角度→int16小端hex"""
    val = int(round(deg * 10))
    b = struct.pack('<h', val)
    print(f"CAN帧字节: {b[0]:02X} {b[1]:02X}")
    print(f"int16值:   {val}")
    print(f"          ↓")
    print(f"cansend can0 130#11 01 {b[0]:02X} {b[1]:02X} 00 00 00 00")

def hex_to_angle(h1, h2):
    """int16小端hex→角度"""
    val = struct.unpack('<h', bytes([h1, h2]))[0]
    deg = val / 10.0
    print(f"int16值: {val}")
    print(f"角度:    {deg}°")
    print(f"        → J1 发: cansend can0 130#11 01 {h1:02X} {h2:02X} 00 00 00 00")
    print(f"        → J2 发: cansend can0 130#11 02 {h1:02X} {h2:02X} 00 00 00 00")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("角度→编码: python3 angle_calc.py <角度>")
        print("编码→角度: python3 angle_calc.py <hex1> <hex2>")
        exit()

    a1 = sys.argv[1]

    # 检测是否带了 --joint 参数
    joint = 1
    if a1 == '--joint' and len(sys.argv) >= 4:
        joint = int(sys.argv[2])
        a1 = sys.argv[3]

    # 尝试转数字判断是角度还是hex
    try:
        if len(sys.argv) >= 3 and not a1.startswith('-'):
            # 可能两个hex
            h1 = int(a1, 16)
            h2 = int(sys.argv[2], 16)
            if 0 <= h1 <= 255 and 0 <= h2 <= 255:
                hex_to_angle(h1, h2)
                exit()
        # 角度转换
        deg = float(a1)
        angle_to_hex(deg)
    except ValueError:
        print(f"无法识别的输入: {a1}")
