"""Detector contracts and dummy implementation."""

from __future__ import annotations

from src.Vision.frame_types import Detection, Frame


class Detector:
    """Abstract detector interface."""

    def detect(self, frame: Frame) -> list[Detection]:
        """Return detections for a frame."""
        raise NotImplementedError


class DummyDetector(Detector):
    """Detector used by dry-run and tests."""

    def __init__(self, *, emit_detection: bool = True, label: str = "dummy") -> None:
        """Create a dummy detector."""
        self._emit_detection = emit_detection
        self._label = label

    def detect(self, frame: Frame) -> list[Detection]:
        """Return one centered detection or an empty list."""
        if not self._emit_detection:
            return []
        width = frame.width * 0.25
        height = frame.height * 0.25
        return [
            Detection(
                label=self._label,
                confidence=1.0,
                cx=frame.width / 2.0,
                cy=frame.height / 2.0,
                width=width,
                height=height,
            )
        ]
