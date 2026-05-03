"""Camera source contracts and basic implementations."""

from __future__ import annotations

import time
from typing import Any

from src.Vision.frame_types import Frame


class CameraSource:
    """Abstract camera source interface."""

    def start(self) -> None:
        """Start the camera source."""
        raise NotImplementedError

    def read(self, timeout_s: float | None = None) -> Frame | None:
        """Read one frame or return None on timeout."""
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the camera source."""
        raise NotImplementedError


class DummyCameraSource(CameraSource):
    """Dry-run camera source that emits placeholder frames."""

    def __init__(self, *, width: int = 640, height: int = 480, fps: int = 30, data: Any = None) -> None:
        """Create a dummy camera source."""
        self._width = width
        self._height = height
        self._fps = fps
        self._data = data
        self._running = False
        self._last_frame_ts = 0.0

    def start(self) -> None:
        """Start the dummy camera source."""
        self._running = True

    def read(self, timeout_s: float | None = None) -> Frame | None:
        """Return a dummy frame while running."""
        if not self._running:
            return None

        min_period_s = 1.0 / self._fps if self._fps > 0 else 0.0
        now = time.monotonic()
        wait_s = max(0.0, min_period_s - (now - self._last_frame_ts))
        if timeout_s is not None and wait_s > timeout_s:
            time.sleep(max(0.0, timeout_s))
            return None
        if wait_s > 0:
            time.sleep(wait_s)

        self._last_frame_ts = time.monotonic()
        return Frame(ts=time.time(), width=self._width, height=self._height, data=self._data)

    def stop(self) -> None:
        """Stop the dummy camera source."""
        self._running = False


class OpenCVCameraSource(CameraSource):
    """OpenCV camera source imported lazily at instantiation time."""

    def __init__(self, *, camera_index: int = 0, width: int = 640, height: int = 480, fps: int = 30) -> None:
        """Create an OpenCV camera source."""
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV is required for OpenCVCameraSource") from exc

        self._cv2 = cv2
        self._camera_index = camera_index
        self._width = width
        self._height = height
        self._fps = fps
        self._capture = None

    def start(self) -> None:
        """Open the OpenCV capture device."""
        capture = self._cv2.VideoCapture(self._camera_index)
        capture.set(self._cv2.CAP_PROP_FRAME_WIDTH, self._width)
        capture.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        capture.set(self._cv2.CAP_PROP_FPS, self._fps)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError("Failed to open OpenCV camera source")
        self._capture = capture

    def read(self, timeout_s: float | None = None) -> Frame | None:
        """Read one frame from OpenCV."""
        if self._capture is None:
            return None
        ok, data = self._capture.read()
        if not ok:
            return None
        height, width = data.shape[:2]
        return Frame(ts=time.time(), width=int(width), height=int(height), data=data)

    def stop(self) -> None:
        """Release the OpenCV capture device."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None
