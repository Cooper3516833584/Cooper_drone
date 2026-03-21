from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class FailsafeAction(str, Enum):
    LAND = "LAND"
    LOITER = "LOITER"
    BRAKE = "BRAKE"


class SafetyState(str, Enum):
    NORMAL = "NORMAL"
    GUIDED_ALLOWED = "GUIDED_ALLOWED"
    KILL = "KILL"
    TAKEOVER_REVOKED = "TAKEOVER_REVOKED"
    LINK_LOST = "LINK_LOST"
    RC_STALE = "RC_STALE"


@dataclass(frozen=True)
class TelemetryView:
    timestamp_monotonic: float
    mode: str
    armed: bool
    lat_deg: float | None
    lon_deg: float | None
    alt_m: float | None
    relative_alt_m: float | None
    vx_mps: float | None
    vy_mps: float | None
    vz_mps: float | None
    battery_voltage_v: float | None
    battery_remaining_pct: int | None
    gps_fix_type: int | None
    satellites_visible: int | None
    rc_channels: dict[int, int] = field(default_factory=dict)
    heartbeat_age_s: float | None = None
    ekf_healthy: bool | None = None
    latest_status_text: str | None = None

