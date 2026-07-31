"""
R1 车控主服务 — 树莓派 4B 部署版
功能：
  1. USB-CAN 本地底盘控制
  2. 外部串口指令解析 (遥控器/上位机)
  3. ROS2 Humble 节点桥接
"""

import serial
import serial.tools.list_ports
import can
import threading
import time
import json
import struct
from dataclasses import dataclass
from typing import Optional, Callable

# ========== 配置 ==========

SERIAL_BAUD = 115200
CAN_BITRATE = 500000

# 串口设备映射（根据实际 /dev/serial/by-id/ 调整）
# 外部指令输入串口（如遥控器接收机）
EXTERNAL_SERIAL_PORT = "/dev/ttyUSB0"
# 下位机 MCU 串口
MCU_SERIAL_PORT = "/dev/ttyUSB1"

# ========== CAN 底盘控制 ==========

class CanChassis:
    """USB-CAN 底盘驱动"""

    def __init__(self, channel: str = "can0"):
        self.bus: Optional[can.Bus] = None
        self.channel = channel

    def start(self):
        try:
            self.bus = can.Bus(
                interface="socketcan",
                channel=self.channel,
                bitrate=CAN_BITRATE
            )
            print(f"[CAN] 已打开 {self.channel}")
        except Exception as e:
            print(f"[CAN] 打开失败: {e}")
            return False
        return True

    def send_motor_speed(self, motor_id: int, speed_rpm: float):
        """发送电机速度指令"""
        speed_int = int(speed_rpm * 10)  # 转为整数，精度 0.1 RPM
        data = struct.pack("<Bh", 0x11, speed_int)  # 0x11 = 速度命令
        msg = can.Message(
            arbitration_id=0x200 + motor_id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)

    def send_chassis_speed(self, linear_x: float, linear_y: float, angular_z: float):
        """底盘速度控制 (m/s, rad/s)"""
        # 打包为 6 字节: vx, vy, wz 各 2 字节 (mm/s, mrad/s)
        vx = int(linear_x * 1000)
        vy = int(linear_y * 1000)
        wz = int(angular_z * 1000)
        data = struct.pack("<Bhhh", 0x10, vx, vy, wz)
        msg = can.Message(
            arbitration_id=0x100,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)
        print(f"[CAN] 底盘速度: vx={linear_x:.2f} vy={linear_y:.2f} wz={angular_z:.2f}")

# ========== 串口指令解析 ==========

@dataclass
class Command:
    """解析后的指令"""
    type: str          # "chassis", "motor", "gripper", "custom"
    cmd: str           # 原始命令字
    params: dict       # 参数字典
    raw: str           # 原始字符串

class SerialCommandParser:
    """外部串口指令解析器"""

    def __init__(self, callback: Callable[[Command], None]):
        self.callback = callback
        self.buffer = ""

    def feed(self, data: str):
        """喂数据给解析器"""
        self.buffer += data
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if line:
                self._parse(line)

    def _parse(self, line: str):
        """解析一行指令"""
        parts = line.split()
        if not parts:
            return

        cmd_type = ""
        params = {}

        # === 指令格式示例 ===
        # MOVE 0.5 0 0        → 前进 0.5m/s
        # TURN 0.3            → 自转 0.3rad/s
        # MOTOR 1 200         → 1号电机 200RPM
        # GRIP 50             → 夹爪 50%
        # STOP                → 停车
        # CUSTOM ...          → 透传自定义指令

        keyword = parts[0].upper()

        if keyword == "MOVE":
            cmd_type = "chassis"
            vx = float(parts[1]) if len(parts) > 1 else 0
            vy = float(parts[2]) if len(parts) > 2 else 0
            wz = float(parts[3]) if len(parts) > 3 else 0
            params = {"vx": vx, "vy": vy, "wz": wz}

        elif keyword == "TURN":
            cmd_type = "chassis"
            params = {"vx": 0, "vy": 0, "wz": float(parts[1])}

        elif keyword == "MOTOR":
            cmd_type = "motor"
            params = {"id": int(parts[1]), "rpm": float(parts[2])}

        elif keyword == "GRIP":
            cmd_type = "gripper"
            params = {"percent": float(parts[1])}

        elif keyword == "STOP":
            cmd_type = "chassis"
            params = {"vx": 0, "vy": 0, "wz": 0}

        elif keyword == "CUSTOM":
            cmd_type = "custom"
            params = {"data": " ".join(parts[1:])}

        cmd = Command(type=cmd_type, cmd=keyword, params=params, raw=line)
        print(f"[SERIAL <<<] {line}")
        self.callback(cmd)

# ========== 主调度 ==========

class R1Controller:
    """R1 车控主控制器"""

    def __init__(self):
        self.chassis = CanChassis()
        self.external_serial: Optional[serial.Serial] = None
        self.mcu_serial: Optional[serial.Serial] = None
        self.parser = SerialCommandParser(self.on_command)
        self.running = False

    def on_command(self, cmd: Command):
        """收到外部指令后的回调"""
        if cmd.type == "chassis":
            self.chassis.send_chassis_speed(
                cmd.params.get("vx", 0),
                cmd.params.get("vy", 0),
                cmd.params.get("wz", 0)
            )
            # 同时也转发给下位机 MCU
            if self.mcu_serial and self.mcu_serial.is_open:
                self.mcu_serial.write((cmd.raw + "\n").encode())

        elif cmd.type == "motor":
            self.chassis.send_motor_speed(
                cmd.params["id"],
                cmd.params["rpm"]
            )

    def serial_reader_thread(self, port: str, baud: int):
        """串口读取线程"""
        while self.running:
            try:
                with serial.Serial(port, baud, timeout=0.05) as ser:
                    print(f"[SERIAL] 已打开 {port}")
                    while self.running:
                        if ser.in_waiting:
                            data = ser.read(ser.in_waiting).decode(errors="ignore")
                            self.parser.feed(data)
                        time.sleep(0.001)
            except serial.SerialException as e:
                print(f"[SERIAL] {port} 错误: {e}")
                time.sleep(2)  # 等待重连

    def list_serial_ports(self):
        """列出所有可用串口"""
        ports = serial.tools.list_ports.comports()
        print("\n=== 可用串口 ===")
        for p in ports:
            print(f"  {p.device} - {p.description}")
        print()

    def start(self):
        self.running = True

        # 1. 启动 CAN
        if not self.chassis.start():
            print("[WARN] CAN 启动失败，将在无 CAN 模式下运行")

        # 2. 列出串口
        self.list_serial_ports()

        # 3. 启动串口读取线程
        ext_thread = threading.Thread(
            target=self.serial_reader_thread,
            args=(EXTERNAL_SERIAL_PORT, SERIAL_BAUD),
            daemon=True
        )
        ext_thread.start()

        print("\n[R1] 主服务已启动")
        print("  等待外部指令...")
        print("  支持指令: MOVE|TURN|MOTOR|GRIP|STOP|CUSTOM")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[R1] 正在关闭...")
            self.stop()

    def stop(self):
        self.running = False
        print("[R1] 已关闭")

# ========== ROS2 节点 ==========

def start_ros2_node():
    """启动 ROS2 节点（可选）"""
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class R1RosNode(Node):
        def __init__(self, controller: R1Controller):
            super().__init__("r1_control")
            self.controller = controller
            self.sub = self.create_subscription(
                Twist, "/cmd_vel", self.cmd_vel_callback, 10
            )

        def cmd_vel_callback(self, msg: Twist):
            self.controller.chassis.send_chassis_speed(
                msg.linear.x, msg.linear.y, msg.angular.z
            )

    rclpy.init()
    node = R1RosNode(controller)
    rclpy.spin(node)


# ========== 启动入口 ==========

if __name__ == "__main__":
    print("=" * 50)
    print("  R1 车控系统 v1.0")
    print("  树莓派 4B + Ubuntu 22.04 + ROS2 Humble")
    print("=" * 50)

    controller = R1Controller()

    # 可选: 在另一个线程启动 ROS2
    # ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
    # ros_thread.start()

    controller.start()
