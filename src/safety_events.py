"""Structured safety event dataclass and recorder.

SafetyEvent is a frozen dataclass that captures every safety-relevant
moment in a machine-readable form.  SafetyEventRecorder buffers events
in memory and optionally forwards them to the RuntimeLogger JSONL stream
(logs/<run_id>/safety.jsonl).

Design constraints
------------------
* Recorder exceptions must never propagate to the caller – a logging
  failure must not affect safety actions.
* Tests use the in-memory list only, no real files required.
* No large payloads (images, frames, model results) are stored here.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import time
from typing import Any

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Event type constants
# ---------------------------------------------------------------------------

EVENT_STATE_TRANSITION = "state_transition"
EVENT_DECISION_EVALUATED = "decision_evaluated"
EVENT_ACTION_STARTED = "action_started"
EVENT_ACTION_COMPLETED = "action_completed"
EVENT_ACTION_FAILED = "action_failed"
EVENT_ACTION_SKIPPED_ONCE_KEY = "action_skipped_once_key"
EVENT_MOTION_INHIBITED = "motion_inhibited"
EVENT_MOTION_CLEARED = "motion_cleared"
EVENT_MISSION_CANCELLED = "mission_cancelled"
EVENT_SUPERVISOR_ERROR = "supervisor_error"


# ---------------------------------------------------------------------------
# SafetyEvent
# ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class SafetyEvent:
    """Immutable record of a single safety-relevant occurrence.

    All fields with a default of ``None`` are optional context that not
    every event type will carry.
    """

    timestamp_monotonic: float
    event_type: str
    state: str
    previous_state: str | None
    reason: str
    heartbeat_age_s: float | None = None
    rc_age_s: float | None = None
    rc_channels: dict[int, int] | None = None
    actions: list[dict[str, Any]] | None = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable dict of this event."""
        return dataclasses.asdict(self)

    def to_json(self) -> str:
        """Return a compact JSON string for this event."""
        return json.dumps(self.to_dict(), ensure_ascii=True, sort_keys=True)


# ---------------------------------------------------------------------------
# SafetyEventRecorder
# ---------------------------------------------------------------------------

class SafetyEventRecorder:
    """Collect SafetyEvents in memory and optionally write to a RuntimeLogger.

    The recorder is intentionally separated from RuntimeLogger so that
    tests can use a plain in-memory list without touching the filesystem.

    Exceptions raised inside :meth:`record` are caught and logged via the
    standard ``logging`` module only – they must not propagate to callers.
    """

    def __init__(self, runtime_logger: Any | None = None) -> None:
        """Create a recorder.

        Parameters
        ----------
        runtime_logger:
            An optional ``RuntimeLogger`` instance (or any object that
            exposes a ``safety(name, **fields)`` method).  When provided,
            every recorded event is also written to the JSONL safety log.
        """
        self._logger = runtime_logger
        self.events: list[SafetyEvent] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def record(self, event: SafetyEvent) -> None:
        """Append *event* to the in-memory list and forward to the logger.

        This method is exception-safe: any error is swallowed and reported
        only through the standard ``logging`` system.
        """
        try:
            self.events.append(event)
            self._forward_to_logger(event)
        except Exception:  # noqa: BLE001
            _log.exception("SafetyEventRecorder.record failed – event dropped")

    def record_state_transition(
        self,
        *,
        state: str,
        previous_state: str | None,
        reason: str,
        heartbeat_age_s: float | None = None,
        rc_age_s: float | None = None,
    ) -> None:
        """Convenience helper: emit a state_transition event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_STATE_TRANSITION,
            state=state,
            previous_state=previous_state,
            reason=reason,
            heartbeat_age_s=heartbeat_age_s,
            rc_age_s=rc_age_s,
        ))

    def record_motion_inhibited(self, *, state: str, reason: str) -> None:
        """Convenience helper: emit a motion_inhibited event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_MOTION_INHIBITED,
            state=state,
            previous_state=None,
            reason=reason,
        ))

    def record_motion_cleared(self, *, state: str, previous_state: str, reason: str) -> None:
        """Convenience helper: emit a motion_cleared event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_MOTION_CLEARED,
            state=state,
            previous_state=previous_state,
            reason=reason,
        ))

    def record_mission_cancelled(self, *, state: str, reason: str) -> None:
        """Convenience helper: emit a mission_cancelled event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_MISSION_CANCELLED,
            state=state,
            previous_state=None,
            reason=reason,
        ))

    def record_action_started(self, *, state: str, action: str, reason: str) -> None:
        """Convenience helper: emit an action_started event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_ACTION_STARTED,
            state=state,
            previous_state=None,
            reason=reason,
            actions=[{"action": action, "reason": reason}],
        ))

    def record_action_completed(
        self,
        *,
        state: str,
        action: str,
        reason: str,
    ) -> None:
        """Convenience helper: emit an action_completed event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_ACTION_COMPLETED,
            state=state,
            previous_state=None,
            reason=reason,
            actions=[{"action": action, "reason": reason}],
        ))

    def record_action_failed(
        self,
        *,
        state: str,
        action: str,
        reason: str,
        error: str,
    ) -> None:
        """Convenience helper: emit an action_failed event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_ACTION_FAILED,
            state=state,
            previous_state=None,
            reason=reason,
            actions=[{"action": action, "reason": reason}],
            error=error,
        ))

    def record_action_skipped(
        self,
        *,
        state: str,
        action: str,
        reason: str,
        once_key: str,
    ) -> None:
        """Convenience helper: emit an action_skipped_once_key event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_ACTION_SKIPPED_ONCE_KEY,
            state=state,
            previous_state=None,
            reason=reason,
            actions=[{"action": action, "reason": reason, "once_key": once_key}],
        ))

    def record_supervisor_error(
        self,
        *,
        state: str,
        reason: str,
        error: str,
    ) -> None:
        """Convenience helper: emit a supervisor_error event."""
        self.record(SafetyEvent(
            timestamp_monotonic=time.monotonic(),
            event_type=EVENT_SUPERVISOR_ERROR,
            state=state,
            previous_state=None,
            reason=reason,
            error=error,
        ))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _forward_to_logger(self, event: SafetyEvent) -> None:
        if self._logger is None:
            return
        if hasattr(self._logger, "safety"):
            self._logger.safety(event.event_type, **event.to_dict())
