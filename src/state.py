"""Thread-safe vehicle state cache."""

from __future__ import annotations

import copy
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from src.mavlink_messages import (
    global_position_to_fields,
    heartbeat_to_mode_and_armed,
    heartbeat_to_system_status,
    message_type,
    rc_channels_to_fields,
    sys_status_to_fields,
)


@dataclass
class VehicleState:
    """Parsed vehicle state fields from MAVLink telemetry."""

    last_heartbeat_ts: float | None = None
    armed: bool | None = None
    mode: str | None = None
    system_status: str | None = None
    relative_alt_m: float | None = None
    lat_deg: float | None = None
    lon_deg: float | None = None
    vx_mps: float | None = None
    vy_mps: float | None = None
    vz_mps: float | None = None
    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None
    rc_last_update_ts: float | None = None
    rc_channels: dict[int, int] = field(default_factory=dict)


class VehicleStateCache:
    """Thread-safe cache for the latest parsed vehicle state."""

    def __init__(self) -> None:
        """Create an empty vehicle state cache."""
        self._lock = threading.Lock()
        self._state = VehicleState()

    def update_from_message(self, msg: Any, now: float | None = None) -> None:
        """Update cached state from a MAVLink message."""
        timestamp = time.time() if now is None else now
        msg_type = message_type(msg)

        with self._lock:
            if msg_type == "HEARTBEAT":
                mode, armed = heartbeat_to_mode_and_armed(msg)
                self._state.last_heartbeat_ts = timestamp
                self._state.mode = mode
                self._state.armed = armed
                self._state.system_status = heartbeat_to_system_status(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._apply_fields(global_position_to_fields(msg))
            elif msg_type == "SYS_STATUS":
                self._apply_fields(sys_status_to_fields(msg))
            elif msg_type == "RC_CHANNELS":
                fields = rc_channels_to_fields(msg)
                self._state.rc_last_update_ts = timestamp
                self._state.rc_channels = fields["rc_channels"]

    def snapshot(self) -> VehicleState:
        """Return a copy of the cached vehicle state."""
        with self._lock:
            return copy.deepcopy(self._state)

    def _apply_fields(self, fields: dict[str, Any]) -> None:
        for key, value in fields.items():
            setattr(self._state, key, value)
