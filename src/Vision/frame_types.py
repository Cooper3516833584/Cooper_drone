"""Frame and tracking data contracts for the Vision package."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class Frame:
    """A camera frame with timestamp and dimensions."""

    ts: float
    width: int
    height: int
    data: Any


@dataclass
class Detection:
    """A normalized target detection in image coordinates."""

    label: str
    confidence: float
    cx: float
    cy: float
    width: float
    height: float


@dataclass
class TrackingOutput:
    """Tracker output with normalized errors and control suggestions."""

    valid: bool
    error_x: float
    error_y: float
    target_area: float
    yaw_rate_cmd_dps: float
    vx_cmd_mps: float
    vy_cmd_mps: float
    vz_cmd_mps: float
