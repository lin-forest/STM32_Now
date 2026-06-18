#!/usr/bin/env python3
"""ChassisController Phase 1 诊断测试脚本"""

import serial, time, sys

DEV = '/dev/ttyACM10'
BAUD = 115200

def send_cmd(cmd_byte, can_id, data_bytes, name=""):
    """发送自定义协议帧: SOF | CMD | CAN_ID(4B LE) | DLC | Data"""
    can_id_le = can_id.to_bytes(4, 'little')
    dlc = len(data_bytes)
    frame = bytes([0xAA, cmd_byte]) + can_id_le + bytes([dlc]) + bytes(data_bytes)

    ser.reset_input_buffer()
    ser.write(frame)
    print(f"[发送] {name or '测试帧'}: {frame.hex().upper()}")

    time.sleep(1.0)
    resp = ser.read(2000)
    if resp:
        text = resp.decode('utf-8', errors='replace')
        for line in text.split('\n'):
            line = line.strip()
            if line:
                print(f"  << {line}")
    else:
        print("  (无返回)")

if __name__ == '__main__':
    ser = serial.Serial(DEV, BAUD, timeout=3, write_timeout=1)
    time.sleep(0.3)

    print(f"串口 {DEV} @ {BAUD} 已打开\n")
    print("=" * 50)

    # === 测试1: CMD_GET_STATE (cmd=0x02) → 期望: STATE_RSP ===
    send_cmd(
        cmd_byte=0x02,          # CMD_GET_STATE
        can_id=0x123,           # 转向电机控制ID
        data_bytes=[0x01],      # 查询命令
        name="CMD_GET_STATE"
    )

    print("=" * 50)

    # === 测试2: 透传帧 (cmd=0xFF) → 期望: CAN_TX OK ===
    send_cmd(
        cmd_byte=0xFF,          # 透传
        can_id=0x125,           # 目标CAN ID
        data_bytes=[0x11, 0x20],# 速度命令 + 速度值
        name="透传 CAN 0x125"
    )

    ser.close()
    print("\n测试完成")
