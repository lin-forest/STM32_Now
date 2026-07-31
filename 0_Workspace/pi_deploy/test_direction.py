#!/usr/bin/env python3
"""
R1 底盘方向测试脚本
====================
逐单元测试每个电机的转向，帮你建立正确的方向映射表。

用法（在树莓派上）:
    python3 test_direction.py

流程:
    1. 逐一测试4个单元的驱动电机（正转/反转）
    2. 逐一测试4个单元的转向电机（正转/反转）
    3. 生成方向映射表
"""

import subprocess
import time
import sys

# ========== CAN ID 映射 ==========
# 单元编号: (驱动CMD ID, 转向CMD ID, 驱动状态ID, 转向状态ID)
UNITS = {
    1: {'power': 0x122, 'turn': 0x121, 'status_power': 0x322, 'status_turn': 0x321},
    2: {'power': 0x124, 'turn': 0x123, 'status_power': 0x324, 'status_turn': 0x323},
    3: {'power': 0x126, 'turn': 0x125, 'status_power': 0x326, 'status_turn': 0x325},
    4: {'power': 0x128, 'turn': 0x127, 'status_power': 0x328, 'status_turn': 0x327},
}

# 预设的单元→物理位置猜测
# Unit 1 → 前左, Unit 2 → 前右, Unit 3 → 后左, Unit 4 → 后右
POSITION_NAMES = {
    1: "前左 (Front-Left)",
    2: "前右 (Front-Right)",
    3: "后左 (Rear-Left)",
    4: "后右 (Rear-Right)",
}


def can_send(can_id, data):
    """发送 CAN 帧"""
    data_str = "#" + "".join(f"{b:02X}" for b in data)
    cmd = f"cansend can0 {can_id:X}{data_str}"
    subprocess.run(cmd, shell=True, capture_output=True)


def stop_all():
    """紧急停止所有电机"""
    print("\n  ⛔ 急停所有电机...")
    can_send(0x101, [0x08, 0, 0, 0, 0, 0, 0, 0])
    for uid in UNITS:
        for mtype in ['power', 'turn']:
            can_send(UNITS[uid][mtype], [0x08, 0, 0, 0, 0, 0, 0, 0])
    time.sleep(0.5)


def test_motor(unit_id, motor_type, speed, duration=2.0):
    """
    测试单个电机

    参数:
        unit_id: 1-4
        motor_type: 'power'(驱动) 或 'turn'(转向)
        speed: -100 ~ 100（正=正转，负=反转）
        duration: 运行秒数
    """
    can_id = UNITS[unit_id][motor_type]
    label = f"Unit{unit_id} {POSITION_NAMES[unit_id]} {'驱动' if motor_type=='power' else '转向'}"
    direction = "正转 (+)" if speed > 0 else "反转 (-)"

    print(f"\n  🔄 测试: {label} | {direction} | speed={speed:+.0f}")
    print(f"     CAN ID: 0x{can_id:X}")

    # 发速度指令
    speed_byte = speed & 0xFF
    can_send(can_id, [0x11, speed_byte, 0, 0, 0, 0, 0, 0])

    # 等待
    time.sleep(duration)
    stop_all()
    time.sleep(1)


def main():
    print("=" * 60)
    print("  R1 底盘方向测试")
    print("=" * 60)
    print()
    print("⚠️  安全提示:")
    print("  1. 将底盘架起来，轮子离地")
    print("  2. 确保周围没有人")
    print("  3. 准备好观察每个轮子的转向方向")
    print()
    input("按 Enter 开始测试...")

    results = {}

    # === 阶段1: 测试驱动电机 ===
    print("\n" + "=" * 60)
    print("  阶段1: 驱动电机测试（正转 +50）")
    print("=" * 60)
    print()
    print("观察每个轮子的转动方向，并记录下来：")
    print("  +  = 轮子向前转（车头方向）")
    print("  -  = 轮子向后转（车尾方向）")
    print("  ?  = 不确定")
    print()

    for uid in range(1, 5):
        test_motor(uid, 'power', 50, duration=2.0)
        direction = input(f"  Unit{uid} ({POSITION_NAMES[uid]}) 驱动电机转向 [+/−/?]: ")
        results[f"U{uid}_power"] = direction.strip()

    # === 阶段2: 测试转向电机 ===
    print("\n" + "=" * 60)
    print("  阶段2: 转向电机测试（正转 +50）")
    print("=" * 60)
    print()
    print("观察每个轮子的转向方向，并记录下来：")
    print("  +  = 逆时针转（从上往下看）")
    print("  -  = 顺时针转")
    print("  ?  = 不确定")
    print()

    for uid in range(1, 5):
        test_motor(uid, 'turn', 50, duration=1.5)
        direction = input(f"  Unit{uid} ({POSITION_NAMES[uid]}) 转向电机旋转方向 [+/−/?]: ")
        results[f"U{uid}_turn"] = direction.strip()

    # === 阶段3: 生成配置 ===
    print("\n" + "=" * 60)
    print("  测试结果 & 方向映射表")
    print("=" * 60)
    print()

    print("你观察到的方向：")
    for uid in range(1, 5):
        pos = POSITION_NAMES[uid]
        p = results.get(f"U{uid}_power", "?")
        t = results.get(f"U{uid}_turn", "?")
        print(f"  Unit{uid} ({pos}):  驱动={p}  转向={t}")

    print()
    print("方向反转配置建议：")
    print("  如果某个轮子的方向反了，需要在代码中反转：")
    print()
    print("  ```python")
    print("  # 在 mclm_can.py 的 set_speed() 中加方向映射")
    print("  DIRECTION_REVERSE = {")
    for uid in range(1, 5):
        p = results.get(f"U{uid}_power", "?")
        t = results.get(f"U{uid}_turn", "?")
        p_rev = "True" if p == "-" else "False"
        t_rev = "True" if t == "-" else "False"
        print(f"      {uid}: {{'power': {p_rev}, 'turn': {t_rev}}},  # {POSITION_NAMES[uid]}")
    print("  }")
    print("  ```")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  ⛔ 测试中断，急停...")
        stop_all()
        sys.exit(0)
