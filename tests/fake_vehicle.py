"""
fake_vehicle — FakeVehicle 模拟器

用于 dry-run 模式和单元测试，模拟 DroneKit Vehicle 的基本接口，
无需真实飞控即可运行主状态机和测试流程。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class _FakeMode:
    """模拟 DroneKit VehicleMode。"""
    name: str = "LOITER"


@dataclass
class _FakeLocation:
    lat: float = 22.5431
    lon: float = 114.0579
    alt: float = 0.0


@dataclass
class _FakeLocationContainer:
    global_relative_frame: _FakeLocation = field(default_factory=_FakeLocation)
    global_frame: _FakeLocation = field(default_factory=_FakeLocation)


@dataclass
class _FakeBattery:
    voltage: float = 12.6
    current: float = 0.0
    level: int = 85


@dataclass
class _FakeGPS:
    fix_type: int = 3
    satellites_visible: int = 12


class _FakeMessageFactory:
    """模拟 DroneKit message_factory。"""

    @staticmethod
    def set_position_target_local_ned_encode(*args, **kwargs):
        """返回空消息对象。"""
        return {"type": "SET_POSITION_TARGET_LOCAL_NED", "args": args}


class FakeVehicle:
    """模拟 DroneKit Vehicle——用于 dry-run 模式和单元测试。

    支持的属性：
        mode, armed, location, velocity, heading,
        battery, gps_0, ekf_ok, channels, last_heartbeat,
        message_factory

    支持的方法：
        simple_takeoff, simple_goto, send_mavlink, close
    """

    def __init__(self) -> None:
        self.mode = _FakeMode("LOITER")
        self.armed = False
        self.location = _FakeLocationContainer()
        self.velocity = [0.0, 0.0, 0.0]
        self.heading = 0
        self.battery = _FakeBattery()
        self.gps_0 = _FakeGPS()
        self.ekf_ok = True
        self.last_heartbeat = 0.0
        self.message_factory = _FakeMessageFactory()

        # RC 通道（字符串键，模拟 DroneKit 行为）
        self.channels: dict[str, int] = {
            "1": 1500,  # Roll
            "2": 1500,  # Pitch
            "3": 1000,  # Throttle
            "4": 1500,  # Yaw
            "5": 1000,  # Mode / 接管（低=Loiter）
            "6": 1500,
            "7": 1000,
            "8": 1000,  # Kill（低=正常）
        }

        self._start_time = time.time()

    def simple_takeoff(self, alt: float) -> None:
        """模拟起飞——立即将高度设置为目标值。"""
        self.location.global_relative_frame.alt = alt

    def simple_goto(self, location) -> None:
        """模拟飞向目标——立即将位置设置为目标值。"""
        if hasattr(location, "lat"):
            self.location.global_relative_frame.lat = location.lat
            self.location.global_relative_frame.lon = location.lon
            self.location.global_relative_frame.alt = location.alt

    def send_mavlink(self, msg) -> None:
        """模拟发送 MAVLink 消息——无操作。"""
        pass

    def close(self) -> None:
        """模拟断开连接。"""
        pass

    @property
    def last_heartbeat(self) -> float:
        """距上次心跳的秒数——FakeVehicle 始终返回 0。"""
        return 0.0

    @last_heartbeat.setter
    def last_heartbeat(self, value: float) -> None:
        pass
