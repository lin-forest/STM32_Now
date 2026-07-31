#!/usr/bin/env python3
"""
舵轮底盘方向映射测试
====================
逐单元测试，找出 CAN ID 到物理轮子的实际映射关系。

用法（树莓派）:
    python3 test_mapping.py

流程:
    Phase 1: 逐单元驱动测试 → 找出 4 个轮子各是哪个 CAN ID
    Phase 2: 转向零位标定  → 手动回正后记录 ticks
    Phase 3: 输出正确的 WHEEL_POS 配置
"""

import subprocess
import time
import sys

# ========== CAN ID 映射 ==========
# 命令 ID 和状态 ID
UNITS = [
    {'name': 'UNIT1', 'power_cmd': 0x122, 'turn_cmd': 0x121, 'turn_status': 0x321, 'power_status': 0x322},
    {'name': 'UNIT2', 'power_cmd': 0x124, 'turn_cmd': 0x123, 'turn_status': 0x323, 'power_status': 0x324},
    {'name': 'UNIT3', 'power_cmd': 0x126, 'turn_cmd': 0x125, 'turn_status': 0x325, 'power_status': 0x326},
    {'name': 'UNIT4', 'power_cmd': 0x128, 'turn_cmd': 0x127, 'turn_status': 0x327, 'power_status': 0x328},
]

def can_send(can_id, data):
    d = "".join(f"{b:02X}" for b in data)
    cmd = f"cansend can0 {can_id:X}#{d}"
    subprocess.run(cmd, shell=True)

def stop_all():
    can_send(0x101, [0x08, 0, 0, 0, 0, 0, 0, 0])
    for u in UNITS:
        can_send(u['power_cmd'], [0x08, 0, 0, 0, 0, 0, 0, 0])
        can_send(u['turn_cmd'], [0x08, 0, 0, 0, 0, 0, 0, 0])
    time.sleep(0.5)

def print_header(s):
    print("\n" + "=" * 65)
    print(f"  {s}")
    print("=" * 65)

# =====================================================================
# Phase 1: 驱动电机测试 → 找出每个 CAN ID 对应的物理轮子
# =====================================================================
def phase1_drive_test():
    print_header("Phase 1: 驱动电机测试 — 找出每个 CAN ID 对应的物理轮子")
    print()
    print("  说明：逐个单元发送 +50 驱动指令，观察哪个轮子转")
    print()
    input("  按 Enter 开始测试（确保轮子离地）...")

    mapping = {}
    for idx, u in enumerate(UNITS):
        print(f"\n  >>> 发送 +50 到 {u['name']} (CAN 0x{u['power_cmd']:X})")
        can_send(u['power_cmd'], [0x11, 0x32, 0, 0, 0, 0, 0, 0])
        time.sleep(2)
        stop_all()
        time.sleep(1)

        ans = input(f"  哪个轮子在转？输入位置 [前左/前右/后左/后右]: ")
        mapping[idx] = ans.strip()
        print(f"  → {u['name']} = {mapping[idx]}")

    print("\n  === 驱动测试结果 ===")
    for idx, pos in mapping.items():
        print(f"    {UNITS[idx]['name']} (0x{UNITS[idx]['power_cmd']:X}) → {pos}")
    return mapping

# =====================================================================
# Phase 2: 转向零位标定
# =====================================================================
def phase2_zero_cal():
    print_header("Phase 2: 转向零位标定")
    print()
    print("  步骤：")
    print("    1. 手动把 4 个轮子都转到正前方（用手推）")
    print("    2. 读当前转向编码器 ticks")
    print("    3. 这些 ticks 值就是零位偏移")
    print()
    input("  ** 请先把所有轮子转到正前方，然后按 Enter...")

    # 先停转向电机
    for u in UNITS:
        can_send(u['turn_cmd'], [0x08, 0, 0, 0, 0, 0, 0, 0])

    time.sleep(0.5)

    print("\n  正在读转向状态帧（等 2 秒收集数据）...")
    # 打印 candump 输出
    print("\n  开另一个终端执行: candump can0,0x320:0x7F0")
    print("  复制 5 秒内的输出粘贴回来")

# =====================================================================
# Phase 3: 转向方向测试
# =====================================================================
def phase3_turn_test():
    print_header("Phase 3: 转向电机方向测试")
    print()
    print("  逐个发送 +50 转向指令，观察轮子朝哪转")
    print()

    for idx, u in enumerate(UNITS):
        print(f"\n  >>> 发送 +50 到 {u['name']} 转向 (CAN 0x{u['turn_cmd']:X})")
        can_send(u['turn_cmd'], [0x11, 0x32, 0, 0, 0, 0, 0, 0])
        time.sleep(1.5)
        stop_all()
        time.sleep(1)

        ans = input(f"  转向方向？[逆时针/顺时针]: ")
        print(f"  → {u['name']} 转向 = {ans}")

# =====================================================================
# Phase 4: 输出最终配置
# =====================================================================
def phase4_config(mapping):
    print_header("Phase 4: 生成正确配置")
    print()
    print("  把以下配置复制到 chassis_control.py:")
    print()

    # 重新排序 WHEEL_POS
    # 标准：四个轮子按 FL, FR, RL, RR 排列
    # 但 DriveUnit 索引必须匹配实际物理轮子

    print("  # 替换 WHEEL_POS 为：")
    print("  WHEEL_POS = [")
    for idx in range(4):
        pos = mapping.get(idx, f"UNIT{idx+1}")
        print(f"      (0.25,  0.30),   # {pos}  (0x{UNITS[idx]['turn_cmd']:X})")

    print("  ]")
    print()
    print("  然后：")
    print("  1. 把轮子转到正前方")
    print("  2. 启动 chassis_control.py")
    print("  3. 输入 'cal' 归零")

if __name__ == "__main__":
    try:
        mapping = phase1_drive_test()
        phase3_turn_test()
        phase4_config(mapping)
    except KeyboardInterrupt:
        print("\n\n  中断")
        stop_all()
