#!/usr/bin/env python3
"""
ROS2 舵轮底盘 CAN 桥接节点
============================
将 ROS2 /cmd_vel 转换为 MCLM CAN 命令直接发给电机控制器。
同时监听 CAN 状态帧，发布底盘状态到 /chassis_status。

架构:  PC (ROS2) ──CAN──▶ MCLM_t2 电机控制器（跳过 ChassisController）

依赖:
  pip install python-can rclpy

用法:
  # Linux socketcan
  sudo ip link set can0 up type can bitrate 1000000
  ros2 run ... chassis_can_node.py --ros-args -p channel:=can0

  # CANable slcan
  ros2 run ... chassis_can_node.py --ros-args -p channel:=slcan0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
import math
import threading
from dataclasses import dataclass

# ── 引入 CAN 协议库 ──
from mclm_can import (
    MCLMCanInterface, DriveUnit, MotorStatus,
    steering_inverse_kinematics,
)


@dataclass
class WheelConfig:
    """轮子安装位置（相对于车体中心，米）"""
    x: float  # 车头方向为正
    y: float  # 左侧方向为正


# ── 四舵轮底盘默认参数 ──
DEFAULT_WHEEL_POSITIONS = [
    WheelConfig(x=0.30, y=0.25),   # 前左
    WheelConfig(x=0.30, y=-0.25),  # 前右
    WheelConfig(x=-0.30, y=0.25),  # 后左
    WheelConfig(x=-0.30, y=-0.25), # 后右
]


class ChassisCanNode(Node):
    """
    ROS2 → CAN 舵轮底盘桥接节点

    订阅:
        /cmd_vel (geometry_msgs/Twist) — 车体速度指令

    发布:
        /chassis_status (String) — 各电机状态 JSON
        /diagnostics (String) — 系统诊断信息
    """

    def __init__(self):
        super().__init__('chassis_can_node')

        # ── 参数 ──
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('bustype', 'socketcan')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('max_speed', 100.0)     # 最大逻辑速度
        self.declare_parameter('max_linear_x', 1.0)    # 最大前进速度 m/s
        self.declare_parameter('max_linear_y', 0.8)    # 最大侧向速度 m/s
        self.declare_parameter('max_angular_z', 1.5)   # 最大旋转角速度 rad/s
        self.declare_parameter('wheel_base_x', 0.60)   # 轴距 m
        self.declare_parameter('wheel_base_y', 0.50)   # 轮距 m
        self.declare_parameter('angle_ctrl_enabled', False)  # 是否开启角度模式
        self.declare_parameter('publish_rate', 20.0)   # 状态发布频率 Hz

        channel = self.get_parameter('channel').value
        bustype = self.get_parameter('bustype').value
        bitrate = self.get_parameter('bitrate').value
        self._max_speed = self.get_parameter('max_speed').value
        self._max_vx = self.get_parameter('max_linear_x').value
        self._max_vy = self.get_parameter('max_linear_y').value
        self._max_omega = self.get_parameter('max_angular_z').value
        self._angle_ctrl = self.get_parameter('angle_ctrl_enabled').value
        self._publish_rate = self.get_parameter('publish_rate').value

        # 构建轮子坐标（四舵轮底盘）
        wx = self.get_parameter('wheel_base_x').value / 2.0
        wy = self.get_parameter('wheel_base_y').value / 2.0
        self._wheel_positions = [
            (wx, wy),   # 前左
            (wx, -wy),  # 前右
            (-wx, wy),  # 后左
            (-wx, -wy), # 后右
        ]

        # ── CAN 接口 ──
        self._can = MCLMCanInterface(
            channel=channel,
            bustype=bustype,
            bitrate=bitrate,
        )

        # 注册状态回调
        self._can.on_status(self._on_motor_status)

        # 打开并启动接收
        try:
            self._can.open()
            self._can.start()
            self.get_logger().info(f'CAN 接口已打开: {channel} @ {bitrate}bps')
        except Exception as e:
            self.get_logger().error(f'CAN 接口打开失败: {e}')
            self.get_logger().error('请确认 CAN 适配器已连接且已配置')
            self.get_logger().error('  Linux: sudo ip link set can0 up type can bitrate 1000000')

        # ── 订阅 ──
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, qos)

        # ── 发布 ──
        self._status_pub = self.create_publisher(String, '/chassis_status', 10)
        self._diag_pub = self.create_publisher(String, '/diagnostics', 10)

        # ── 定时器 ──
        period = 1.0 / max(self._publish_rate, 1.0)
        self.create_timer(period, self._publish_status)

        # ── 状态缓存 ──
        self._latest_status: dict[str, MotorStatus] = {}
        self._lock = threading.Lock()

        # ── 上次 cmd_vel 时间（超时自动停止） ──
        self._last_cmd_time = time.monotonic()
        self._cmd_timeout = 1.0  # 1秒无命令自动停止

        self.get_logger().info('底盘 CAN 节点已启动')
        self.get_logger().info(f'  角度模式: {"开启" if self._angle_ctrl else "关闭"}')
        self.get_logger().info(f'  最大速度: vx={self._max_vx} vy={self._max_vy} omega={self._max_omega}')

    # ────────────────────────────────────────────────
    # /cmd_vel 回调
    # ────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        """处理 /cmd_vel 速度指令"""
        # 安全检查：超速保护
        vx = max(-self._max_vx, min(self._max_vx, msg.linear.x))
        vy = max(-self._max_vy, min(self._max_vy, msg.linear.y))
        omega = max(-self._max_omega, min(self._max_omega, msg.angular.z))

        # 运动学逆解 → 各单元速度+角度
        wheel_cmds = steering_inverse_kinematics(vx, vy, omega, self._wheel_positions)

        for unit_id, cmd in enumerate(wheel_cmds):
            drive_speed = cmd['drive_speed']
            steer_angle = cmd['steer_angle_deg']

            # 速度归一化到 -100~100
            speed_norm = max(-100, min(100, int(drive_speed / self._max_vx * self._max_speed)))

            if self._angle_ctrl:
                # 角度模式: 发送目标角度（需 MCLM 支持）
                self._can.set_steering_angle(DriveUnit(unit_id), steer_angle)
                # 速度模式: 驱动电机正常发速度
                self._can.set_speed(DriveUnit(unit_id), 'power', speed_norm)
            else:
                # 纯速度模式: 转向电机发速度值
                # 注意：此处转向电机的速度值应该映射为"转向的快慢"而非位置
                # 实际上需要 MCLM 的角度闭环才能真正实现"打方向到目标角度"
                turn_speed = self._angle_to_turn_speed(steer_angle, unit_id)
                self._can.set_speed(DriveUnit(unit_id), 'turn', turn_speed)
                self._can.set_speed(DriveUnit(unit_id), 'power', speed_norm)

        self._last_cmd_time = time.monotonic()

    def _angle_to_turn_speed(self, target_angle: float, unit_id: int) -> int:
        """
        将目标角度转为转向电机速度指令（仅纯速度模式用）

        这是在没有角度闭环情况下的简易方案：
        - 根据当前角度和目标角度的差值决定转向方向和速度
        - 如果当前角度未知（第一次），用默认速度
        """
        status = self._can.get_status(DriveUnit(unit_id), 'turn')
        current_angle = status.current_angle if status else 0.0

        # 计算最短路径误差
        error = (target_angle - current_angle + 180) % 360 - 180

        # PID 比例控制
        kp = 1.5
        speed = int(error * kp)
        return max(-80, min(80, speed))

    # ────────────────────────────────────────────────
    # CAN 状态回调
    # ────────────────────────────────────────────────

    def _on_motor_status(self, status: MotorStatus):
        """收到电机状态帧"""
        with self._lock:
            key = f'u{status.unit_id}_{status.motor_type}'
            self._latest_status[key] = status

    # ────────────────────────────────────────────────
    # 定时发布
    # ────────────────────────────────────────────────

    def _publish_status(self):
        """发布底盘状态"""
        # cmd_vel 超时检测
        if time.monotonic() - self._last_cmd_time > self._cmd_timeout:
            self._can.stop_all()
            self.get_logger().warn('cmd_vel 超时，已自动停止')

        with self._lock:
            if not self._latest_status:
                return
            statuses = dict(self._latest_status)

        # 构建状态消息
        lines = []
        for unit_id in range(4):
            turn = statuses.get(f'u{unit_id}_turn')
            power = statuses.get(f'u{unit_id}_power')

            if turn:
                stall = '⚠️' if turn.stall else '✅'
                lines.append(
                    f'UNIT{unit_id+1} 转向: '
                    f'speed={turn.current_speed:+.0f} '
                    f'angle={turn.current_angle:.1f}° '
                    f'pwm={turn.pwm} '
                    f'{stall}'
                )
            if power:
                lines.append(
                    f'UNIT{unit_id+1} 驱动: '
                    f'speed={power.current_speed:+.0f} '
                    f'pwm={power.pwm} '
                    f'{"⚠️" if power.stall else "✅"}'
                )

        msg = String()
        msg.data = '\n'.join(lines)
        self._status_pub.publish(msg)

    # ────────────────────────────────────────────────
    # 节点生命周期
    # ────────────────────────────────────────────────

    def destroy_node(self):
        self._can.stop_all()
        self._can.close()
        super().destroy_node()


# ── 入口 ──
def main(args=None):
    import time
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
