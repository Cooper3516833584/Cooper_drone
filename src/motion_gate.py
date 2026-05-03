"""Thread-safe motion gate for low-level movement commands."""

from __future__ import annotations

import threading


class MotionInhibitedError(RuntimeError):
    """Raised when motion is blocked by the motion gate."""


class MotionGate:
    """Thread-safe gate that can inhibit movement commands."""

    def __init__(self) -> None:
        """Create an open motion gate."""
        self._lock = threading.RLock()
        self._reason: str | None = None

    def inhibit(self, reason: str) -> None:
        """Block movement commands with a reason."""
        with self._lock:
            self._reason = reason

    def clear(self) -> None:
        """Allow movement commands again."""
        with self._lock:
            self._reason = None

    def is_inhibited(self) -> bool:
        """Return whether movement is currently blocked."""
        with self._lock:
            return self._reason is not None

    def reason(self) -> str:
        """Return the current inhibit reason."""
        with self._lock:
            return self._reason or ""

    def assert_motion_allowed(self) -> None:
        """Raise if normal motion output is currently blocked."""
        with self._lock:
            reason = self._reason
        if reason is not None:
            raise MotionInhibitedError(reason)

    def assert_allowed(self) -> None:
        """Raise if movement commands are currently blocked."""
        self.assert_motion_allowed()
