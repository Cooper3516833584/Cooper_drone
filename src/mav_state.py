from __future__ import annotations

import copy
import threading
import time
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class PositionState:
    lat_deg: float | None = None
    lon_deg: float | None = None
    alt_m: float | None = None
    relative_alt_m: float | None = None
    heading_deg: float | None = None


@dataclass
class VelocityState:
    vx_mps: float | None = None
    vy_mps: float | None = None
    vz_mps: float | None = None


@dataclass
class BatteryState:
    voltage_v: float | None = None
    current_a: float | None = None
    remaining_pct: int | None = None


@dataclass
class RcState:
    channels: dict[int, int] = field(default_factory=dict)
    last_update_monotonic: float | None = None


@dataclass
class VehicleState:
    mode: str = "UNKNOWN"
    armed: bool = False
    system_status: int | None = None
    gps_fix_type: int | None = None
    satellites_visible: int | None = None
    ekf_healthy: bool | None = None

    position: PositionState = field(default_factory=PositionState)
    velocity: VelocityState = field(default_factory=VelocityState)
    battery: BatteryState = field(default_factory=BatteryState)
    rc: RcState = field(default_factory=RcState)

    home_lat_deg: float | None = None
    home_lon_deg: float | None = None
    home_alt_m: float | None = None

    latest_status_text: str | None = None
    latest_status_severity: int | None = None

    last_heartbeat_monotonic: float | None = None
    last_rc_update_monotonic: float | None = None
    last_update_monotonic: float | None = None


class VehicleStateCache:
    """线程安全状态缓存。任务层只读 snapshot，不直接改内部字段。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = VehicleState()

    def snapshot(self) -> VehicleState:
        with self._lock:
            return copy.deepcopy(self._state)

    def update(self, mutator: Callable[[VehicleState], None]) -> None:
        with self._lock:
            mutator(self._state)
            self._state.last_update_monotonic = time.monotonic()

    def update_heartbeat(self, *, mode: str, armed: bool, system_status: int | None) -> None:
        now = time.monotonic()
        with self._lock:
            self._state.mode = mode
            self._state.armed = armed
            self._state.system_status = system_status
            self._state.last_heartbeat_monotonic = now
            self._state.last_update_monotonic = now

    def update_position(
        self,
        *,
        lat_deg: float | None,
        lon_deg: float | None,
        alt_m: float | None,
        relative_alt_m: float | None,
        heading_deg: float | None,
    ) -> None:
        with self._lock:
            self._state.position.lat_deg = lat_deg
            self._state.position.lon_deg = lon_deg
            self._state.position.alt_m = alt_m
            self._state.position.relative_alt_m = relative_alt_m
            self._state.position.heading_deg = heading_deg
            self._state.last_update_monotonic = time.monotonic()

    def update_velocity(self, *, vx_mps: float | None, vy_mps: float | None, vz_mps: float | None) -> None:
        with self._lock:
            self._state.velocity.vx_mps = vx_mps
            self._state.velocity.vy_mps = vy_mps
            self._state.velocity.vz_mps = vz_mps
            self._state.last_update_monotonic = time.monotonic()

    def update_battery(self, *, voltage_v: float | None, current_a: float | None, remaining_pct: int | None) -> None:
        with self._lock:
            if voltage_v is not None:
                self._state.battery.voltage_v = voltage_v
            if current_a is not None:
                self._state.battery.current_a = current_a
            if remaining_pct is not None:
                self._state.battery.remaining_pct = remaining_pct
            self._state.last_update_monotonic = time.monotonic()

    def update_gps(self, *, fix_type: int | None, satellites_visible: int | None) -> None:
        with self._lock:
            self._state.gps_fix_type = fix_type
            self._state.satellites_visible = satellites_visible
            self._state.last_update_monotonic = time.monotonic()

    def update_ekf_health(self, *, ekf_healthy: bool | None) -> None:
        with self._lock:
            self._state.ekf_healthy = ekf_healthy
            self._state.last_update_monotonic = time.monotonic()

    def update_rc_channels(self, channels: dict[int, int]) -> None:
        now = time.monotonic()
        with self._lock:
            self._state.rc.channels = dict(channels)
            self._state.rc.last_update_monotonic = now
            self._state.last_rc_update_monotonic = now
            self._state.last_update_monotonic = now

    def update_home(self, *, lat_deg: float | None, lon_deg: float | None, alt_m: float | None) -> None:
        with self._lock:
            self._state.home_lat_deg = lat_deg
            self._state.home_lon_deg = lon_deg
            self._state.home_alt_m = alt_m
            self._state.last_update_monotonic = time.monotonic()

    def update_status_text(self, *, severity: int | None, text: str) -> None:
        with self._lock:
            self._state.latest_status_severity = severity
            self._state.latest_status_text = text
            self._state.last_update_monotonic = time.monotonic()

