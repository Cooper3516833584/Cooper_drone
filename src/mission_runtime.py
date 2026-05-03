"""Mission cancellation and runner helpers."""

from __future__ import annotations

import threading
from typing import Callable

from src.logging_runtime import RuntimeLogger


class MissionCancelled(RuntimeError):
    """Raised when a mission cancellation token is cancelled."""


class CancellationToken:
    """Thread-safe mission cancellation token."""

    def __init__(self) -> None:
        """Create an uncancelled token."""
        self._lock = threading.Lock()
        self._reason: str | None = None

    def cancel(self, reason: str) -> None:
        """Cancel the token with a reason."""
        with self._lock:
            if self._reason is None:
                self._reason = reason

    def is_cancelled(self) -> bool:
        """Return whether the token has been cancelled."""
        with self._lock:
            return self._reason is not None

    def reason(self) -> str | None:
        """Return the cancellation reason."""
        with self._lock:
            return self._reason

    def raise_if_cancelled(self) -> None:
        """Raise if the token has been cancelled."""
        reason = self.reason()
        if reason is not None:
            raise MissionCancelled(reason)


def run_mission(
    name: str,
    fn: Callable[[CancellationToken], None],
    logger: RuntimeLogger,
    teardown: Callable[[], None] | None = None,
) -> int:
    """Run a cancellable mission function and write mission lifecycle events."""
    token = CancellationToken()
    logger.event("mission_started", mission=name)
    try:
        fn(token)
        token.raise_if_cancelled()
    except MissionCancelled as exc:
        logger.event("mission_cancelled", mission=name, reason=str(exc))
        return 2
    except Exception as exc:
        logger.event("mission_failed", mission=name, error=repr(exc))
        return 1
    finally:
        if teardown is not None:
            try:
                teardown()
                logger.event("mission_teardown_finished", mission=name)
            except Exception as exc:
                logger.event("mission_teardown_failed", mission=name, error=repr(exc))

    logger.event("mission_finished", mission=name, status="ok")
    return 0
