#!/usr/bin/env python3
"""
J0 第三关节位置控制 — PC 端简易角度闭环
=========================================
基于 TIM2 编码器 ticks 推算角度，P 控制器让 DC 电机转到目标角度。

用法:  python3 j0_control.py

数字命令直接设目标角度:
  90      → 转到 90°
  -45     → 转到 -45°
  0       → 回正

其他命令:
  s 30    → 速度模式 speed=30
  a       → 切回角度模式
  cal     → 当前位置归零
  stop    → 停止
  q       → 退出

协议:
  RX 0x330 状态帧 → bytes[2-3] = 编码器原始值 (uint16 LE)
  TX 0x130 命令帧 → [0x11, 0x00, speed_L, speed_H, 0,0,0,0]
"""
import sys, time, threading, struct, can

# ── 参数 ──
STATUS_ID  = 0x330       # 状态上报 ID
CMD_ID     = 0x130       # 命令 ID
JOINT_ID   = 0x00        # J0
KP         = 0.8         # P 增益
MAX_SPEED  = 100         # 最大逻辑速度
DEAD_ZONE  = 3.0         # 到位死区 (°)
TICKS_PER_REV = 48       # 编码器 ticks/圈 (J0_ENCODER_TICK_PER_REV)

# ── 全局 ──
cur_raw   = 0            # CAN 帧原始 uint16
abs_ticks = 0            # 解包后连续 int32
prev_raw  = None
zero_offset = 0          # cal 归零偏移
target_angle = 0.0
angle_mode = True
manual_speed = 0
running = True


def unwrap(raw):
    """uint16 → 连续 int32：检测 65535→0 或 0→65535 跳变"""
    global abs_ticks, prev_raw
    if prev_raw is None:
        abs_ticks = raw
    else:
        diff = raw - prev_raw
        if diff > 32768:
            diff -= 65536
        elif diff < -32768:
            diff += 65536
        abs_ticks += diff
    prev_raw = raw


def ticks_to_deg(t):
    """ticks → 角度 (0~360)"""
    return (t % TICKS_PER_REV) / TICKS_PER_REV * 360.0


def shortest_error(current, target):
    """最短路径误差 [-180, 180)"""
    return (target - current + 180) % 360 - 180


def cli_thread():
    """键盘输入线程"""
    global target_angle, angle_mode, manual_speed, zero_offset, running
    while running:
        try:
            line = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line == 'q':
            running = False
            break
        elif line == 'stop':
            print(">> 停止")
        elif line == 'a':
            angle_mode = True
            print(f">> 角度模式, 目标={target_angle:.0f}°")
        elif line == 'cal':
            zero_offset = abs_ticks
            target_angle = 0.0
            angle_mode = True
            print(f">> 已归零 (offset={zero_offset})")
        elif line.startswith('s '):
            try:
                manual_speed = int(line[2:])
            except ValueError:
                print(">> s <speed>")
                continue
            angle_mode = False
            print(f">> 速度模式 speed={manual_speed}")
        else:
            try:
                target_angle = float(line) % 360
                angle_mode = True
                print(f">> 目标角度 = {target_angle:.0f}°")
            except ValueError:
                print(">> 数字=设角度 | s 30=速度 | cal=归零 | stop | q")


def send_speed(bus, speed):
    """发送 J0 速度命令 (CAN 0x130)"""
    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
    data = bytes([0x11, JOINT_ID, speed & 0xFF, (speed >> 8) & 0xFF, 0, 0, 0, 0])
    msg = can.Message(arbitration_id=CMD_ID, data=data)
    bus.send(msg)


def main():
    global cur_raw, abs_ticks, target_angle, angle_mode, manual_speed, running

    bus = can.interface.Bus('can0', interface='socketcan')

    # 启动输入线程
    t = threading.Thread(target=cli_thread, daemon=True)
    t.start()

    print("=" * 60)
    print("  J0 第三关节位置控制  (CAN 0x330 ← 0x130)")
    print(f"  ticks/圈={TICKS_PER_REV}  Kp={KP}  max={MAX_SPEED}")
    print("=" * 60)
    print("  数字=设角度 | cal=归零 | s 30=速度 | a=角度 | stop | q")
    print("-" * 60)

    last_sent_speed = 999
    last_display = 0

    try:
        while running:
            # ── 接收状态帧 ──
            msg = bus.recv(timeout=0.05)
            if msg and msg.arbitration_id == STATUS_ID and len(msg.data) >= 4:
                cur_raw = msg.data[2] | (msg.data[3] << 8)
                unwrap(cur_raw)

            # ── 控制计算 ──
            send = None
            if angle_mode:
                angle = ticks_to_deg(abs_ticks - zero_offset)
                err = shortest_error(angle, target_angle)
                if abs(err) < DEAD_ZONE:
                    send = 0
                else:
                    speed = int(err * KP)
                    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
                    # 克服静摩擦
                    if 0 < abs(speed) < 6:
                        speed = 6 if speed > 0 else -6
                    send = speed
            else:
                send = manual_speed

            # ── 发送（去重） ──
            if send is not None and send != last_sent_speed:
                send_speed(bus, send)
                last_sent_speed = send

            # ── 显示（每 0.2s） ──
            now = time.time()
            if now - last_display >= 0.2:
                angle = ticks_to_deg(abs_ticks - zero_offset)
                err = shortest_error(angle, target_angle) if angle_mode else 0
                s = send if send is not None else last_sent_speed
                mode = 'A' if angle_mode else 'S'
                print(f"  [{mode}] 角度={angle:6.1f}°  目标={target_angle:6.1f}°  "
                      f"误差={err:+6.1f}°  speed={s:+3d}  raw={cur_raw:5d}  "
                      f"abs={abs_ticks:6d}" + ' ' * 10, end='\r')
                last_display = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n停止...")
        send_speed(bus, 0)
        bus.shutdown()


if __name__ == '__main__':
    main()
