#!/usr/bin/env python3
"""
MCLM CAN 直连通信测试脚本（无 ROS2 依赖）
=============================================
直接通过 CAN 总线发送命令给 MCLM_t2 电机控制器，接收并显示状态帧。

用法:
  # 基本测试
  python3 test_direct_can.py --channel can0

  # 交互模式
  python3 test_direct_can.py --channel can0 --interactive

  # 快速前进 2 秒
  python3 test_direct_can.py --channel can0 --go 0.5

依赖:
  pip install python-can
"""

import argparse
import time
import sys
import threading

# 添加当前目录到 path 以便导入 mclm_can
sys.path.insert(0, '.')
from mclm_can import MCLMCanInterface, DriveUnit, MotorStatus


def parse_args():
    parser = argparse.ArgumentParser(description='MCLM CAN 直连测试')
    parser.add_argument('--channel', default='can0', help='CAN 接口 (default: can0)')
    parser.add_argument('--bustype', default='socketcan', help='CAN 总线类型 (default: socketcan)')
    parser.add_argument('--bitrate', type=int, default=1000000, help='波特率 (default: 1000000)')
    parser.add_argument('--interactive', '-i', action='store_true', help='交互模式')
    parser.add_argument('--go', type=float, help='快速前进指定秒数后停止')
    parser.add_argument('--dump', action='store_true', help='持续监听 CAN 总线')
    return parser.parse_args()


def print_status(status: MotorStatus):
    """打印电机状态"""
    icon = '🛑' if status.stall else '✅' if not status.saturated else '⚠️'
    extra = ''
    if status.motor_type == 'turn' and status.angle_mode:
        extra = f' 角度={status.current_angle:.1f}° (目标={status.target_angle:.0f}°)'
    elif status.motor_type == 'turn' and status.current_angle > 0:
        extra = f' 角度≈{status.current_angle:.1f}°'

    print(
        f'[{icon}] UNIT{status.unit_id+1} {status.motor_type:5s} | '
        f'speed={status.current_speed:+.0f}  '
        f'target={status.target_speed:+.0f}  '
        f'pwm={status.pwm:5d}  '
        f'{"STALL " if status.stall else ""}'
        f'{"SAT " if status.saturated else ""}'
        f'{extra}'
    )


def interactive_mode(can: MCLMCanInterface):
    """交互式控制"""
    import readline  # 命令行编辑支持

    print("\n" + "=" * 60)
    print("交互模式，输入命令:")
    print("  s <unit> <speed>      — 设置转向电机速度，unit=1~4, speed=-100~100")
    print("  p <unit> <speed>      — 设置驱动电机速度")
    print("  a <unit> <angle>      — 设置转向角度（需角度模式）")
    print("  m <unit> <0|1>        — 切换模式 0=速度 1=角度")
    print("  stop [unit]           — 停止单个或全部电机")
    print("  q                     — 查询所有电机状态")
    print("  dump                  — 持续监听 CAN 总线")
    print("  h                     — 帮助")
    print("  exit/quit             — 退出")
    print("=" * 60)

    @can.on_status
    def _on_status(status):
        pass  # 已注册回调，不额外处理

    try:
        while True:
            try:
                line = input("can> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ('exit', 'quit'):
                break

            elif cmd == 's':
                unit = DriveUnit(int(parts[1]) - 1) if len(parts) > 1 else DriveUnit.UNIT_1
                speed = int(parts[2]) if len(parts) > 2 else 0
                can.set_speed(unit, 'turn', speed)
                print(f"→ UNIT{unit+1} 转向: speed={speed}")

            elif cmd == 'p':
                unit = DriveUnit(int(parts[1]) - 1) if len(parts) > 1 else DriveUnit.UNIT_1
                speed = int(parts[2]) if len(parts) > 2 else 0
                can.set_speed(unit, 'power', speed)
                print(f"→ UNIT{unit+1} 驱动: speed={speed}")

            elif cmd == 'a':
                unit = DriveUnit(int(parts[1]) - 1) if len(parts) > 1 else DriveUnit.UNIT_1
                angle = float(parts[2]) if len(parts) > 2 else 90.0
                can.set_steering_angle(unit, angle)
                print(f"→ UNIT{unit+1} 转向: angle={angle}°")

            elif cmd == 'm':
                unit = DriveUnit(int(parts[1]) - 1) if len(parts) > 1 else DriveUnit.UNIT_1
                enable = bool(int(parts[2])) if len(parts) > 2 else True
                can.enable_angle_mode(unit, enable)
                print(f"→ UNIT{unit+1} 角度模式={'ON' if enable else 'OFF'}")

            elif cmd == 'stop':
                if len(parts) > 1:
                    unit = DriveUnit(int(parts[1]) - 1)
                    can.stop_motor(unit, 'turn')
                    can.stop_motor(unit, 'power')
                    print(f"→ UNIT{unit+1} 已停止")
                else:
                    can.stop_all()
                    print("→ 全车急停")

            elif cmd == 'q':
                statuses = can.get_all_status()
                if not statuses:
                    print("(暂无状态数据)")
                for key, status in statuses.items():
                    print_status(status)

            elif cmd == 'dump':
                print("持续监听 CAN 总线（Ctrl+C 停止）...")

                @can.on_status
                def _dump(status):
                    ts = time.strftime('%H:%M:%S', time.localtime(status.timestamp))
                    print(f'[{ts}] ', end='')
                    print_status(status)

                try:
                    while True:
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n停止监听")

            elif cmd == 'h':
                print("命令帮助:")
                print("  s <unit> <speed>      speed: -100~100")
                print("  p <unit> <speed>      speed: -100~100")
                print("  a <unit> <angle>      angle: 0~360")
                print("  m <unit> <0|1>        0=速度模式 1=角度模式")
                print("  stop [unit]           不带 unit=全车急停")
                print("  q                     查询所有")
                print("  dump                  持续监听")
                print("  exit/quit             退出")
            else:
                print(f"未知命令: {cmd} (输入 h 查看帮助)")

    except KeyboardInterrupt:
        pass


def go_test(can: MCLMCanInterface, duration: float):
    """前进测试"""
    print(f"\n→ 前进测试: 全车速度 50%, {duration} 秒")

    # 广播前进
    for unit in DriveUnit:
        can.set_speed(unit, 'turn', 0)          # 转向回正
        can.set_speed(unit, 'power', 50)        # 驱动向前 50%

    print("  运行中... 按 Ctrl+C 停止")
    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print("\n  手动中断")

    print("  停止")
    can.stop_all()


def dump_mode(can: MCLMCanInterface):
    """持续监听模式"""
    print(f"\n持续监听 CAN 总线 (Ctrl+C 停止)...")

    @can.on_status
    def handler(status):
        ts = time.strftime('%H:%M:%S', time.localtime(status.timestamp))
        print(f'[{ts}] ', end='')
        print_status(status)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n停止监听")


def main():
    args = parse_args()

    print(f"打开 CAN: {args.channel} @ {args.bitrate}bps")
    can = MCLMCanInterface(
        channel=args.channel,
        bustype=args.bustype,
        bitrate=args.bitrate,
    )

    try:
        can.open()
        can.start()
    except Exception as e:
        print(f"❌ CAN 打开失败: {e}")
        sys.exit(1)

    print("✅ CAN 已连接\n")

    # 注册状态显示回调
    @can.on_status
    def show_status(status):
        ts = time.strftime('%H:%M:%S', time.localtime(status.timestamp))
        print(f'[{ts}] ', end='')
        print_status(status)

    try:
        if args.dump:
            dump_mode(can)
        elif args.go:
            go_test(can, args.go)
        elif args.interactive:
            interactive_mode(can)
        else:
            # 默认：快速测试
            print("=" * 60)
            print("默认测试序列：")
            print("  1. 查询电机状态（会收到一帧回复）")
            print('  2. 前进 2 秒')
            print("  3. 后退 2 秒")
            print("  4. 停止")
            print("  5. 查看状态")
            print("=" * 60)

            time.sleep(1)

            # 查询
            print("\n[1] 查询所有电机状态...")
            for unit in DriveUnit:
                can.query_status(unit, 'turn')
                can.query_status(unit, 'power')
            time.sleep(0.5)

            # 前进
            print("\n[2] 前进: 驱动电机 50%...")
            for unit in DriveUnit:
                can.set_speed(unit, 'power', 50)
            time.sleep(2)

            # 停止
            print("\n[3] 停止...")
            can.stop_all()
            time.sleep(0.5)

            # 后退
            print("\n[4] 后退: 驱动电机 -50%...")
            for unit in DriveUnit:
                can.set_speed(unit, 'power', -50)
            time.sleep(2)

            # 停止
            print("\n[5] 停止...")
            can.stop_all()
            time.sleep(0.5)

            print("\n✅ 测试完成")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        print("停止所有电机...")
        can.stop_all()
        can.close()
        print("再见")


if __name__ == '__main__':
    main()
