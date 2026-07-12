#!/usr/bin/env python3
"""
MCLM CAN 协议封装库
=====================
直接对接 3_MCLM_t2 电机控制器的 CAN 协议，支持舵轮底盘4个动力单元。

硬件要求：
  - PC 端 CAN 适配器（CANable/PCAN/USB2CAN 等）
  - Linux: socketcan 接口 (can0/slcan0)
  - CAN 总线波特率 1Mbps

用法：
    from mclm_can import MCLMCanInterface, DriveUnit

    can = MCLMCanInterface(channel='can0')
    can.set_speed(DriveUnit.UNIT_1, 'turn', 50)      # 单元1 转向电机 50%
    can.set_speed(DriveUnit.UNIT_1, 'power', -30)     # 单元1 驱动电机 -30%
    can.set_steering_angle(DriveUnit.UNIT_2, 90.0)    # 单元2 转向到 90°（需 MCLM 支持角度模式）
    can.stop_all()                                     # 紧急停止全车

    @can.on_status
    def handler(status):
        print(f"单元{status.unit_id} {status.motor_type}: "
              f"速度={status.current_speed}, 角度={status.current_angle}°")

    can.start()  # 启动后台接收线程

参考文档:
  - doc/deepseek_can.md     — CAN 协议详述
  - doc/Function/Func1_planMt6701.md — 角度闭环实施计划
  - doc/chassis_model.md    — 底盘构型设计
"""

import struct
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Optional


# ═══════════════════════════════════════════════════════════════
# CAN ID 定义
# ═══════════════════════════════════════════════════════════════

class CanId(IntEnum):
    """广播命令 ID（所有 Group 共用）"""
    STOP_ALL      = 0x101   # 全车停止
    TURN_BROADCAST = 0x102  # 全车转向命令
    POWER_BROADCAST = 0x103 # 全车动力命令


# ── 每个动力单元的 CAN ID 组 ──
# 索引 0→3 对应单元 1→4
UNIT_CAN_IDS = [
    {  # Unit 1 — CAN_ID_GROUP=3
        'turn_cmd':   0x121,
        'power_cmd':  0x122,
        'turn_status': 0x321,
        'power_status': 0x322,
    },
    {  # Unit 2 — CAN_ID_GROUP=2（默认出厂）
        'turn_cmd':   0x123,
        'power_cmd':  0x124,
        'turn_status': 0x323,
        'power_status': 0x324,
    },
    {  # Unit 3 — CAN_ID_GROUP=1
        'turn_cmd':   0x125,
        'power_cmd':  0x126,
        'turn_status': 0x325,
        'power_status': 0x326,
    },
    {  # Unit 4 — CAN_ID_GROUP=4
        'turn_cmd':   0x127,
        'power_cmd':  0x128,
        'turn_status': 0x327,
        'power_status': 0x328,
    },
]


class DriveUnit(IntEnum):
    """动力单元编号（1-based，对应车体安装位置）"""
    UNIT_1 = 0
    UNIT_2 = 1
    UNIT_3 = 2
    UNIT_4 = 3


# ── 命令字节 ──
CMD_SET_SPEED      = 0x11   # 设置速度，data[1]=speed (-100~100)
CMD_STOP           = 0x08   # 立即停止
CMD_REVERSE        = 0x02   # 倒转（固定速度）
CMD_QUERY_STATUS   = 0x01   # 查询电机状态（即时响应）
CMD_LOG_START      = 0x04   # 开始日志
CMD_LOG_STOP       = 0x05   # 停止日志

# 角度控制命令（需 MCLM 实现 Func1_planMt6701.md 后才可用）
CMD_SET_ANGLE      = 0x12   # 设置目标角度，data[2-3]=angle_x10 LE
CMD_SET_ANGLE_MODE = 0x13   # 切换控制模式，data[4]=0(速度)/1(角度)


# ── 状态帧标志位 ──
FLAG_STALL     = 0x01  # bit0: 堵转
FLAG_SATURATED = 0x02  # bit1: PWM 饱和
FLAG_ANGLE_MODE = 0x80  # bit7: 角度模式（扩展状态帧）


# ── 状态帧索引（用于扩展帧解析） ──
IDX_SPEED_L  = 0
IDX_SPEED_H  = 1
IDX_TICKS_L  = 2
IDX_TICKS_H  = 3
IDX_PWM_L    = 4
IDX_PWM_H    = 5
IDX_TARGET   = 6
IDX_FLAGS    = 7


# ═══════════════════════════════════════════════════════════════
# 数据结构
# ═══════════════════════════════════════════════════════════════

@dataclass
class MotorStatus:
    """单电机状态（从 CAN 状态帧解析）"""
    unit_id: int           # 0~3
    motor_type: str        # 'turn' 或 'power'
    current_speed: float   # 当前速度 (-100~100)
    current_angle: float   # 当前角度（度），仅转向电机有效
    pwm: int               # 当前 PWM 输出值
    target_speed: float    # 目标速度 (-100~100)
    target_angle: float    # 目标角度（度），仅转向电机有效
    stall: bool            # 堵转标志
    saturated: bool        # 饱和标志
    angle_mode: bool       # 是否处于角度模式
    timestamp: float       # 收到时间戳


@dataclass
class ChassisState:
    """车体级状态（运动学正解后）"""
    vx: float = 0.0       # 前进速度 m/s
    vy: float = 0.0       # 侧向速度 m/s
    omega: float = 0.0    # 旋转角速度 rad/s
    units: dict = field(default_factory=dict)  # {unit_id: {turn: MotorStatus, power: MotorStatus}}


# ═══════════════════════════════════════════════════════════════
# 主接口类
# ═══════════════════════════════════════════════════════════════

class MCLMCanInterface:
    """
    MCLM CAN 协议封装

    参数:
        channel: CAN 接口名 (Linux: 'can0', 'slcan0'; Windows: 'PCAN_USBBUS1')
        bustype: python-can 总线类型 ('socketcan', 'pcan', 'serial', 'seeedstudio')
        bitrate: 波特率 (默认 1000000 = 1Mbps)
        can_fd: 是否启用 CAN FD (默认 False)
    """

    def __init__(
        self,
        channel: str = 'can0',
        bustype: str = 'socketcan',
        bitrate: int = 1000000,
        can_fd: bool = False,
    ):
        self._channel = channel
        self._bustype = bustype
        self._bitrate = bitrate
        self._can_fd = can_fd
        self._bus = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._status_callbacks: list[Callable[[MotorStatus], None]] = []
        self._raw_callbacks: list[Callable[[int, bytes], None]] = []
        self._latest_status: dict[tuple[int, str], MotorStatus] = {}
        self._lock = threading.Lock()

    # ────────────────────────────────────────────────
    # 生命周期
    # ────────────────────────────────────────────────

    def open(self):
        """打开 CAN 总线"""
        import can  # 延迟导入，允许在没有 python-can 时导入本文件
        self._bus = can.interface.Bus(
            channel=self._channel,
            bustype=self._bustype,
            bitrate=self._bitrate,
            fd=self._can_fd,
        )
        return self

    def close(self):
        """关闭 CAN 总线"""
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2.0)
        if self._bus:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *args):
        self.close()

    def start(self):
        """启动后台接收线程"""
        if self._bus is None:
            self.open()
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def stop(self):
        """停止后台接收线程"""
        self._running = False

    # ────────────────────────────────────────────────
    # 发送 CAN 命令
    # ────────────────────────────────────────────────

    def send(self, can_id: int, data: bytes):
        """发送原始 CAN 帧"""
        import can
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False,
            is_fd=self._can_fd,
        )
        self._bus.send(msg)

    def set_speed(self, unit: DriveUnit, motor_type: str, speed: int):
        """
        设置电机速度

        参数:
            unit: DriveUnit 枚举
            motor_type: 'turn' 或 'power'
            speed: -100~100（负值反转）
        """
        speed = max(-100, min(100, speed))
        speed_byte = speed & 0xFF  # 转为 uint8（MCLM 端当 int8 解析）
        can_ids = UNIT_CAN_IDS[unit]
        can_id = can_ids['turn_cmd'] if motor_type == 'turn' else can_ids['power_cmd']
        data = bytes([CMD_SET_SPEED, speed_byte, 0, 0, 0, 0, 0, 0])
        self.send(can_id, data)

    def stop_motor(self, unit: DriveUnit, motor_type: str):
        """停止单个电机"""
        can_ids = UNIT_CAN_IDS[unit]
        can_id = can_ids['turn_cmd'] if motor_type == 'turn' else can_ids['power_cmd']
        self.send(can_id, bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))

    def stop_all(self):
        """紧急停止全车（广播 0x101）"""
        self.send(CanId.STOP_ALL, bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))
        # 冗余保护：再发一次
        self.send(CanId.STOP_ALL, bytes([CMD_STOP, 0, 0, 0, 0, 0, 0, 0]))

    def broadcast_speed(self, speed: int):
        """
        广播速度命令到所有驱动电机（0x103）

        适用于简单遥控场景，不需要单个电机独立控制
        """
        speed = max(-100, min(100, speed))
        self.send(CanId.POWER_BROADCAST, bytes([CMD_SET_SPEED, speed & 0xFF, 0, 0, 0, 0, 0, 0]))

    def broadcast_turn(self, speed: int):
        """
        广播转向命令到所有转向电机（0x102）
        """
        speed = max(-100, min(100, speed))
        self.send(CanId.TURN_BROADCAST, bytes([CMD_SET_SPEED, speed & 0xFF, 0, 0, 0, 0, 0, 0]))

    def query_status(self, unit: DriveUnit, motor_type: str):
        """查询单个电机状态（会立即收到一帧状态回复）"""
        can_ids = UNIT_CAN_IDS[unit]
        can_id = can_ids['turn_cmd'] if motor_type == 'turn' else can_ids['power_cmd']
        self.send(can_id, bytes([CMD_QUERY_STATUS, 0, 0, 0, 0, 0, 0, 0]))

    # ── 角度控制（需 MCLM 实现 Func1_planMt6701.md） ──

    def set_steering_angle(self, unit: DriveUnit, angle_deg: float):
        """
        设置转向角度（度）

        要求 MCLM_t2 已实现 Func1_planMt6701.md 的角度闭环控制。
        如果 MCLM 尚未实现，此命令无效。
        """
        # 归一化到 0~360°
        angle_deg = angle_deg % 360.0
        # 转为 x10 整数
        angle_x10 = int(round(angle_deg * 10)) & 0xFFFF
        can_id = UNIT_CAN_IDS[unit]['turn_cmd']
        data = bytes([
            CMD_SET_ANGLE,
            0,                     # motor_id
            angle_x10 & 0xFF,      # angle_L
            (angle_x10 >> 8) & 0xFF,  # angle_H
            0, 0, 0, 0,
        ])
        self.send(can_id, data)

    def enable_angle_mode(self, unit: DriveUnit, enable: bool = True):
        """
        切换角度/速度控制模式

        True  → 角度模式（基于 MT6701 编码器位置闭环）
        False → 速度模式（原始 PWM/编码器速度闭环）
        """
        can_id = UNIT_CAN_IDS[unit]['turn_cmd']
        data = bytes([
            CMD_SET_ANGLE_MODE,
            0,                     # motor_id
            0, 0, 0,
            1 if enable else 0,    # mode
            0, 0,
        ])
        self.send(can_id, data)

    # ────────────────────────────────────────────────
    # 回调注册
    # ────────────────────────────────────────────────

    def on_status(self, callback: Callable[[MotorStatus], None]):
        """注册电机状态回调（每收到一帧状态帧触发）"""
        self._status_callbacks.append(callback)
        return callback

    def on_raw(self, callback: Callable[[int, bytes], None]):
        """注册原始 CAN 帧回调（收到任意 CAN 帧触发）"""
        self._raw_callbacks.append(callback)
        return callback

    # ────────────────────────────────────────────────
    # 读取最新状态
    # ────────────────────────────────────────────────

    def get_status(self, unit: DriveUnit, motor_type: str = 'turn') -> Optional[MotorStatus]:
        """获取某个电机的最新状态"""
        key = (int(unit), motor_type)
        with self._lock:
            return self._latest_status.get(key)

    def get_all_status(self) -> dict:
        """获取所有电机的最新状态"""
        with self._lock:
            return dict(self._latest_status)

    # ────────────────────────────────────────────────
    # 内部
    # ────────────────────────────────────────────────

    def _rx_loop(self):
        """后台接收循环"""
        while self._running:
            try:
                msg = self._bus.recv(timeout=0.5)
                if msg is None:
                    continue
            except Exception:
                if self._running:
                    time.sleep(0.01)
                continue

            can_id = msg.arbitration_id
            data = msg.data

            # 原始帧回调
            for cb in self._raw_callbacks:
                try:
                    cb(can_id, data)
                except Exception:
                    pass

            # 解析状态帧 (0x32x)
            if (can_id & 0xF00) == 0x300 and len(data) >= 8:
                status = self._decode_status_frame(can_id, data)
                if status:
                    key = (status.unit_id, status.motor_type)
                    with self._lock:
                        self._latest_status[key] = status
                    for cb in self._status_callbacks:
                        try:
                            cb(status)
                        except Exception:
                            pass

    @staticmethod
    def _decode_status_frame(can_id: int, data: bytes) -> Optional[MotorStatus]:
        """解码 CAN 状态帧"""
        # 确定单元 ID
        unit_id = None
        motor_type = None
        for uid, ids in enumerate(UNIT_CAN_IDS):
            if can_id == ids['turn_status']:
                unit_id = uid
                motor_type = 'turn'
                break
            if can_id == ids['power_status']:
                unit_id = uid
                motor_type = 'power'
                break

        if unit_id is None:
            return None

        # 解析字节
        speed_raw = struct.unpack_from('<h', data, 0)[0]  # int16 LE
        ticks_raw = struct.unpack_from('<H', data, 2)[0]  # uint16 LE
        pwm_raw   = struct.unpack_from('<h', data, 4)[0]  # int16 LE
        target    = struct.unpack_from('<b', data, 6)[0]   # int8
        flags     = data[7]

        # 检查是否是扩展状态帧（转向电机角度模式）
        # 如果是角度模式，[2-3] 是角度而非 ticks
        angle_mode = bool(flags & FLAG_ANGLE_MODE)
        current_angle = 0.0
        target_angle = 0.0

        if angle_mode and motor_type == 'turn':
            angle_raw = struct.unpack_from('<H', data, 2)[0]  # uint16 LE, 0.1°
            current_angle = angle_raw / 10.0
            target_angle = float(data[6])  # 1° 精度

        return MotorStatus(
            unit_id=unit_id,
            motor_type=motor_type,
            current_speed=float(speed_raw),
            current_angle=current_angle,
            pwm=pwm_raw,
            target_speed=float(target),
            target_angle=target_angle,
            stall=bool(flags & FLAG_STALL),
            saturated=bool(flags & FLAG_SATURATED),
            angle_mode=angle_mode,
            timestamp=time.time(),
        )


# ═══════════════════════════════════════════════════════════════
# 舵轮底盘运动学（PC 端解算）
# ═══════════════════════════════════════════════════════════════

def steering_inverse_kinematics(
    vx: float, vy: float, omega: float,
    wheel_positions: list[tuple[float, float]],
) -> list[dict]:
    """
    舵轮底盘运动学逆解

    将车体速度 (vx, vy, omega) 分解为每个动力单元的驱动速度和转向角度。

    参数:
        vx: 前进速度 (m/s)
        vy: 侧向速度 (m/s)
        omega: 旋转角速度 (rad/s)
        wheel_positions: 每个轮子相对于车体中心的 (x, y) 坐标列表,
                         x 正向 = 车头, y 正向 = 左侧

    返回:
        [{'drive_speed': float, 'steer_angle_deg': float}, ...]
        每个元素对应一个动力单元

    公式（标准舵轮运动学）:
        对于轮 i 在位置 (xi, yi):
        Vix = vx - omega * yi
        Viy = vy + omega * xi
        drive_speed = sqrt(Vix² + Viy²)        （驱动电机速度标量）
        steer_angle = atan2(Viy, Vix) × 180/π   （转向角度，度）
    """
    import math

    results = []
    for x, y in wheel_positions:
        v_ix = vx - omega * y
        v_iy = vy + omega * x

        drive_speed = math.hypot(v_ix, v_iy)
        steer_angle = math.degrees(math.atan2(v_iy, v_ix))

        results.append({
            'drive_speed': drive_speed,
            'steer_angle_deg': steer_angle,
        })

    return results
