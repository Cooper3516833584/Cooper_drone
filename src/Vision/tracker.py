"""Target tracking contracts and simple implementations."""

from __future__ import annotations

from src.Vision.frame_types import Detection, Frame, TrackingOutput


class TargetTracker:
    """Abstract target tracker interface."""

    def update(self, frame: Frame, detections: list[Detection]) -> TrackingOutput:
        """Return tracking output for the latest frame and detections."""
        raise NotImplementedError


class CenteringTracker(TargetTracker):
    """Simple tracker that suggests yaw to center the largest detection."""

    def __init__(self, *, yaw_gain_dps: float = 30.0) -> None:
        """Create a centering tracker."""
        self._yaw_gain_dps = yaw_gain_dps

    def update(self, frame: Frame, detections: list[Detection]) -> TrackingOutput:
        """Compute normalized image error from the largest detection."""
        if not detections:
            return TrackingOutput(
                valid=False,
                error_x=0.0,
                error_y=0.0,
                target_area=0.0,
                yaw_rate_cmd_dps=0.0,
                vx_cmd_mps=0.0,
                vy_cmd_mps=0.0,
                vz_cmd_mps=0.0,
            )

        detection = max(detections, key=lambda item: item.width * item.height)
        center_x = frame.width / 2.0
        center_y = frame.height / 2.0
        error_x = _clamp((detection.cx - center_x) / center_x if center_x else 0.0)
        error_y = _clamp((detection.cy - center_y) / center_y if center_y else 0.0)
        target_area = (detection.width * detection.height) / max(1.0, frame.width * frame.height)

        return TrackingOutput(
            valid=True,
            error_x=error_x,
            error_y=error_y,
            target_area=target_area,
            yaw_rate_cmd_dps=error_x * self._yaw_gain_dps,
            vx_cmd_mps=0.0,
            vy_cmd_mps=0.0,
            vz_cmd_mps=0.0,
        )


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))
