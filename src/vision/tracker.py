from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TrackOutput:
    detected: bool
    err_x: float = 0.0
    err_y: float = 0.0
    distance_m: float | None = None


class TargetTracker:
    """视觉追踪器占位。后续可替换为真实目标检测与跟踪。"""

    def update(self, frame) -> TrackOutput:
        del frame
        return TrackOutput(detected=False)

