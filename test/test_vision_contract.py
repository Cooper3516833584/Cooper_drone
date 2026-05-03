"""Tests for the Vision package contract."""

from __future__ import annotations

import importlib
import logging
import time
from pathlib import Path

from src.Vision.camera import DummyCameraSource
from src.Vision.detector import DummyDetector
from src.Vision.frame_types import Detection, Frame
from src.Vision.tracker import CenteringTracker
from src.Vision.vision_service import VisionService
from src.logging_runtime import RuntimeLogger


def test_dummy_camera_source_produces_frame() -> None:
    """Read a frame from the dummy camera source."""
    camera = DummyCameraSource(width=320, height=240, fps=1000)
    camera.start()
    try:
        frame = camera.read(timeout_s=0.1)
    finally:
        camera.stop()

    assert frame is not None
    assert frame.width == 320
    assert frame.height == 240


def test_dummy_detector_can_return_detection_or_empty_list() -> None:
    """Use dummy detector in enabled and disabled modes."""
    frame = Frame(ts=1.0, width=640, height=480, data=None)

    detections = DummyDetector(emit_detection=True).detect(frame)
    empty = DummyDetector(emit_detection=False).detect(frame)

    assert len(detections) == 1
    assert detections[0].label == "dummy"
    assert empty == []


def test_centering_tracker_produces_tracking_output() -> None:
    """Compute normalized tracking output from a detection."""
    frame = Frame(ts=1.0, width=640, height=480, data=None)
    detection = Detection(label="target", confidence=0.9, cx=640.0, cy=240.0, width=64.0, height=48.0)

    output = CenteringTracker(yaw_gain_dps=30.0).update(frame, [detection])

    assert output.valid is True
    assert output.error_x == 1.0
    assert output.error_y == 0.0
    assert output.yaw_rate_cmd_dps == 30.0


def test_vision_service_start_stop() -> None:
    """Start and stop the background vision service."""
    logger = MemoryRuntimeLogger()
    service = VisionService(
        DummyCameraSource(width=320, height=240, fps=1000),
        DummyDetector(),
        CenteringTracker(),
        logger,
    )

    service.start()
    _wait_until(lambda: service.get_latest_output() is not None)
    service.stop()

    assert service.get_latest_output() is not None
    assert any(event["name"] == "vision_service_started" for event in logger.events)
    assert any(event["name"] == "vision_service_stopped" for event in logger.events)


def test_camera_module_imports_without_opencv() -> None:
    """Import the camera module without instantiating OpenCV camera."""
    module = importlib.import_module("src.Vision.camera")

    assert hasattr(module, "OpenCVCameraSource")


def test_vision_source_does_not_contain_pymavlink_string() -> None:
    """Keep Vision source independent from MAVLink transport."""
    for path in Path("src/Vision").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "pymavlink" not in source


class MemoryRuntimeLogger(RuntimeLogger):
    """Runtime logger that stores Vision events in memory."""

    def __init__(self) -> None:
        """Create an in-memory runtime logger."""
        super().__init__(
            run_id="test-run",
            run_dir=Path("logs/test-vision"),
            app_logger=logging.getLogger("test.vision"),
        )
        self.events: list[dict] = []

    def event(self, name: str, **fields) -> None:
        """Capture an event."""
        self.events.append({"name": name, "fields": fields})


def _wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for predicate")
