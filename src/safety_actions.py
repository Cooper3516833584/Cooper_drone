"""Safety action execution through the low-level movement layer."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SafetyActionResult:
    """Result of one attempted safety action."""

    action: str
    reason: str
    ok: bool
    error: str | None = None
    timestamp_monotonic: float | None = None


class SafetyActionExecutor:
    """Execute safety actions without exposing raw MAVLink to safety policy."""

    def __init__(
        self,
        session: Any,
        logger: Any | None = None,
        *,
        allow_disarm: bool = False,
        allow_force_disarm: bool = False,
    ) -> None:
        """Create an executor around the existing control or movement layer."""
        self._session = session
        self._logger = logger
        self._allow_disarm = allow_disarm
        self._allow_force_disarm = allow_force_disarm

    def execute(self, action: str, *, reason: str) -> SafetyActionResult:
        """Execute one named safety action and return a structured result."""
        normalized = action.lower()
        if normalized == "none":
            return self._record_result("none", reason, True)
        if normalized == "stop":
            return self.stop_motion(reason=reason)
        if normalized == "land":
            return self.land(reason=reason)
        if normalized == "loiter":
            return self.loiter(reason=reason)
        if normalized == "brake":
            return self.brake(reason=reason)
        if normalized == "rtl":
            return self.rtl(reason=reason)
        if normalized == "disarm":
            return self.disarm(reason=reason, force=False)
        if normalized == "force_disarm":
            return self.disarm(reason=reason, force=True)
        return self._record_result(normalized, reason, False, f"Unsupported safety action: {action}")

    def stop_motion(self, *, reason: str) -> SafetyActionResult:
        """Send a zero-motion command through the movement layer."""
        return self._call_action("stop", reason, self._session.stop_motion)

    def land(self, *, reason: str) -> SafetyActionResult:
        """Request landing without waiting for complete touchdown."""
        if hasattr(self._session, "land_nowait"):
            return self._call_action("land", reason, self._session.land_nowait)
        return self._call_action("land", reason, self._session.land)

    def loiter(self, *, reason: str) -> SafetyActionResult:
        """Request LOITER mode through the control layer."""
        if hasattr(self._session, "loiter"):
            return self._call_action("loiter", reason, self._session.loiter)
        return self._set_mode("loiter", "LOITER", reason)

    def brake(self, *, reason: str) -> SafetyActionResult:
        """Request BRAKE mode through the control layer."""
        if hasattr(self._session, "brake"):
            return self._call_action("brake", reason, self._session.brake)
        return self._set_mode("brake", "BRAKE", reason)

    def rtl(self, *, reason: str) -> SafetyActionResult:
        """Request RTL mode when the current movement layer supports it."""
        if hasattr(self._session, "rtl"):
            return self._call_action("rtl", reason, self._session.rtl)
        if hasattr(self._session, "set_mode_nowait") or hasattr(self._session, "set_mode"):
            return self._set_mode("rtl", "RTL", reason)
        return self._record_result("rtl", reason, False, "RTL is not supported by the control layer")

    def disarm(self, *, reason: str, force: bool = False) -> SafetyActionResult:
        """Request disarm only when explicitly enabled for safety actions."""
        action = "force_disarm" if force else "disarm"
        if force:
            self._critical("Force disarm safety action requested: %s", reason)
            if not self._allow_force_disarm:
                return self._record_result(action, reason, False, "Force disarm is not enabled")
        elif not self._allow_disarm:
            return self._record_result(action, reason, False, "Disarm is not enabled as a safety action")

        if hasattr(self._session, "force_disarm") and force:
            return self._call_action(action, reason, self._session.force_disarm)
        return self._call_action(action, reason, self._session.disarm)

    def _set_mode(self, action: str, mode: str, reason: str) -> SafetyActionResult:
        if hasattr(self._session, "set_mode_nowait"):
            return self._call_action(action, reason, self._session.set_mode_nowait, mode)
        if hasattr(self._session, "set_mode"):
            return self._call_action(action, reason, self._session.set_mode, mode)
        return self._record_result(action, reason, False, f"{mode} mode is not supported by the control layer")

    def _call_action(self, action: str, reason: str, fn: Any, *args: Any) -> SafetyActionResult:
        try:
            fn(*args)
        except Exception as exc:
            return self._record_result(action, reason, False, repr(exc))
        return self._record_result(action, reason, True)

    def _record_result(
        self,
        action: str,
        reason: str,
        ok: bool,
        error: str | None = None,
    ) -> SafetyActionResult:
        result = SafetyActionResult(
            action=action,
            reason=reason,
            ok=ok,
            error=error,
            timestamp_monotonic=time.monotonic(),
        )
        self._log_result(result)
        return result

    def _log_result(self, result: SafetyActionResult) -> None:
        fields = {
            "action": result.action,
            "reason": result.reason,
            "ok": result.ok,
            "error": result.error,
            "timestamp_monotonic": result.timestamp_monotonic,
        }
        if hasattr(self._logger, "safety"):
            self._logger.safety("safety_action_result", **fields)
            return
        if isinstance(self._logger, logging.Logger):
            if result.ok:
                self._logger.info("Safety action succeeded: %s reason=%s", result.action, result.reason)
            else:
                self._logger.error(
                    "Safety action failed: %s reason=%s error=%s",
                    result.action,
                    result.reason,
                    result.error,
                )

    def _critical(self, message: str, *args: Any) -> None:
        if hasattr(self._logger, "app_logger"):
            self._logger.app_logger.critical(message, *args)
        elif isinstance(self._logger, logging.Logger):
            self._logger.critical(message, *args)
