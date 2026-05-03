"""Background vision processing service."""

from __future__ import annotations

import threading
import time

from src.Vision.camera import CameraSource
from src.Vision.detector import Detector
from src.Vision.frame_types import TrackingOutput
from src.Vision.tracker import TargetTracker
from src.logging_runtime import RuntimeLogger


class VisionService:
    """Background service that converts frames into tracking output."""

    def __init__(
        self,
        camera: CameraSource,
        detector: Detector,
        tracker: TargetTracker,
        logger: RuntimeLogger,
    ) -> None:
        """Create a vision service."""
        self._camera = camera
        self._detector = detector
        self._tracker = tracker
        self._logger = logger
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_output: TrackingOutput | None = None

    def start(self) -> None:
        """Start the background vision service."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._camera.start()
        self._thread = threading.Thread(target=self._run, name="vision-service", daemon=False)
        self._thread.start()
        self._logger.event("vision_service_started")

    def get_latest_output(self) -> TrackingOutput | None:
        """Return the latest tracking output."""
        with self._lock:
            return self._latest_output

    def stop(self) -> None:
        """Stop the background vision service."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self._logger.event("vision_service_stop_timeout")
        self._camera.stop()
        self._thread = None
        self._logger.event("vision_service_stopped")

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._camera.read(timeout_s=0.1)
                if frame is None:
                    continue
                detections = self._detector.detect(frame)
                output = self._tracker.update(frame, detections)
                with self._lock:
                    self._latest_output = output
                self._logger.event(
                    "vision_tracking_updated",
                    valid=output.valid,
                    error_x=output.error_x,
                    error_y=output.error_y,
                    target_area=output.target_area,
                )
            except Exception as exc:
                self._logger.event("vision_service_error", error=repr(exc))
                time.sleep(0.05)
