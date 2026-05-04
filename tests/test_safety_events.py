"""Tests for SafetyEvent and SafetyEventRecorder (Task 08).

All six minimum requirements from Task 08 are covered:
1. SafetyEvent can be converted to dict and JSON.
2. State changes recorded by SafetySupervisor emit state_transition.
3. Action success emits action_completed.
4. Action failure emits action_failed.
5. once_key deduplication emits action_skipped_once_key.
6. Tests use an in-memory recorder — no real file paths.
"""

from __future__ import annotations

import json

import pytest

from src.config_loader import SafetyConfig
from src.motion_gate import MotionGate
from src.safety import SafetySupervisor, evaluate_safety
from src.safety_events import (
    EVENT_ACTION_COMPLETED,
    EVENT_ACTION_FAILED,
    EVENT_ACTION_SKIPPED_ONCE_KEY,
    EVENT_MISSION_CANCELLED,
    EVENT_MOTION_INHIBITED,
    EVENT_STATE_TRANSITION,
    SafetyEvent,
    SafetyEventRecorder,
)
from src.safety_policy import SafetyState
from src.state import VehicleState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _cfg(**overrides: object) -> SafetyConfig:
    values: dict = {
        "enabled": True,
        "allow_arm": False,
        "require_guided": True,
        "require_heartbeat": True,
        "heartbeat_loss_timeout_s": 1.0,
        "rc_stale_timeout_s": 1.0,
        "manual_takeover_enabled": True,
        "kill_switch_enabled": True,
        "failsafe_action": "land",
    }
    values.update(overrides)
    return SafetyConfig(**values)


def _make_event(event_type: str = EVENT_STATE_TRANSITION) -> SafetyEvent:
    return SafetyEvent(
        timestamp_monotonic=1.0,
        event_type=event_type,
        state="kill",
        previous_state="normal",
        reason="test reason",
    )


class MemorySafetyLogger:
    """Logger stub that captures safety records in memory."""

    def __init__(self) -> None:
        self.safety_records: list[dict] = []
        self.app_logger = self
        self.exceptions: list[str] = []
        self.errors: list[str] = []

    def safety(self, name: str, **fields: object) -> None:
        self.safety_records.append({"name": name, **fields})

    def exception(self, message: str, *args: object) -> None:
        self.exceptions.append(message % args if args else message)

    def error(self, message: str, *args: object) -> None:
        self.errors.append(message % args if args else message)


class RecordingActionExecutor:
    """Fake safety action executor."""

    def __init__(self, *, ok: bool = True) -> None:
        self.calls: list[tuple[str, str]] = []
        self._ok = ok
        self._current_state: str = "unknown"

    def execute(self, action: str, *, reason: str) -> object:
        self.calls.append((action, reason))
        if not self._ok:
            raise RuntimeError("fake executor failure")
        from src.safety_actions import SafetyActionResult
        return SafetyActionResult(action=action, reason=reason, ok=True)


# ---------------------------------------------------------------------------
# 1. SafetyEvent serialisation
# ---------------------------------------------------------------------------

class TestSafetyEventSerialisation:

    def test_to_dict_returns_all_fields(self) -> None:
        """SafetyEvent.to_dict() must be a plain dict with every field."""
        evt = _make_event()
        result = evt.to_dict()
        assert isinstance(result, dict)
        assert result["event_type"] == EVENT_STATE_TRANSITION
        assert result["state"] == "kill"
        assert result["previous_state"] == "normal"
        assert result["reason"] == "test reason"
        assert result["timestamp_monotonic"] == 1.0

    def test_to_json_is_valid_json(self) -> None:
        """SafetyEvent.to_json() must produce parseable JSON."""
        evt = _make_event()
        parsed = json.loads(evt.to_json())
        assert parsed["event_type"] == EVENT_STATE_TRANSITION

    def test_optional_fields_default_to_none(self) -> None:
        """Optional context fields should default to None."""
        evt = _make_event()
        assert evt.heartbeat_age_s is None
        assert evt.rc_age_s is None
        assert evt.rc_channels is None
        assert evt.actions is None
        assert evt.error is None

    def test_to_json_with_all_optional_fields(self) -> None:
        """All optional fields appear in the JSON output when populated."""
        evt = SafetyEvent(
            timestamp_monotonic=2.0,
            event_type=EVENT_ACTION_FAILED,
            state="link_lost",
            previous_state="normal",
            reason="hb timeout",
            heartbeat_age_s=3.5,
            rc_age_s=2.1,
            rc_channels={5: 1800, 7: 1900},
            actions=[{"action": "land", "reason": "hb timeout"}],
            error="TimeoutError()",
        )
        parsed = json.loads(evt.to_json())
        assert parsed["heartbeat_age_s"] == pytest.approx(3.5)
        assert parsed["rc_channels"] == {"5": 1800, "7": 1900}
        assert parsed["error"] == "TimeoutError()"


# ---------------------------------------------------------------------------
# 2. SafetyEventRecorder — in-memory usage
# ---------------------------------------------------------------------------

class TestSafetyEventRecorder:

    def test_record_appends_to_events_list(self) -> None:
        """record() adds the event to the in-memory events list."""
        rec = SafetyEventRecorder()
        evt = _make_event()
        rec.record(evt)
        assert rec.events == [evt]

    def test_recorder_does_not_propagate_exceptions(self) -> None:
        """A buggy logger must not cause record() to raise."""
        class BrokenLogger:
            def safety(self, name: str, **fields: object) -> None:
                raise RuntimeError("logger is broken")

        rec = SafetyEventRecorder(runtime_logger=BrokenLogger())
        # Must not raise even if the logger explodes.
        rec.record(_make_event())
        assert len(rec.events) == 1

    def test_recorder_forwards_to_runtime_logger(self) -> None:
        """Events are forwarded to RuntimeLogger.safety() when provided."""
        mem_logger = MemorySafetyLogger()
        rec = SafetyEventRecorder(runtime_logger=mem_logger)
        evt = _make_event(EVENT_STATE_TRANSITION)
        rec.record(evt)

        assert len(mem_logger.safety_records) == 1
        assert mem_logger.safety_records[0]["name"] == EVENT_STATE_TRANSITION

    def test_convenience_helpers_produce_correct_event_types(self) -> None:
        """Each convenience helper sets the right event_type."""
        rec = SafetyEventRecorder()
        rec.record_state_transition(state="kill", previous_state="normal", reason="r")
        rec.record_motion_inhibited(state="kill", reason="r")
        rec.record_mission_cancelled(state="kill", reason="r")
        rec.record_action_started(state="kill", action="land", reason="r")
        rec.record_action_completed(state="kill", action="land", reason="r")
        rec.record_action_failed(state="kill", action="land", reason="r", error="e")
        rec.record_action_skipped(state="kill", action="land", reason="r", once_key="k")

        types = [e.event_type for e in rec.events]
        assert EVENT_STATE_TRANSITION in types
        assert EVENT_MOTION_INHIBITED in types
        assert EVENT_MISSION_CANCELLED in types
        assert EVENT_ACTION_COMPLETED in types
        assert EVENT_ACTION_FAILED in types
        assert EVENT_ACTION_SKIPPED_ONCE_KEY in types


# ---------------------------------------------------------------------------
# 2. State change → state_transition event via SafetySupervisor
# ---------------------------------------------------------------------------

class TestSupervisorStateTransitionEvent:

    def test_state_transition_event_emitted_on_state_change(self) -> None:
        """SafetySupervisor emits state_transition when the state changes."""
        state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
        cfg = _cfg(link_lost_action="land")
        recorder = SafetyEventRecorder()
        supervisor = SafetySupervisor(
            lambda: state,
            cfg,
            MotionGate(),
            MemorySafetyLogger(),
            lambda reason: None,
            recorder=recorder,
        )
        decision = evaluate_safety(state, cfg, now=100.0)
        supervisor._apply_decision(decision, None)

        transitions = [e for e in recorder.events if e.event_type == EVENT_STATE_TRANSITION]
        assert len(transitions) == 1
        assert transitions[0].state == SafetyState.LINK_LOST.value
        assert transitions[0].previous_state is None

    def test_no_duplicate_transition_events_when_state_unchanged(self) -> None:
        """No state_transition event when the state does not change."""
        state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
        cfg = _cfg(link_lost_action="land")
        recorder = SafetyEventRecorder()
        supervisor = SafetySupervisor(
            lambda: state,
            cfg,
            MotionGate(),
            MemorySafetyLogger(),
            lambda reason: None,
            recorder=recorder,
        )
        decision = evaluate_safety(state, cfg, now=100.0)
        supervisor._apply_decision(decision, SafetyState.LINK_LOST)  # same state

        transitions = [e for e in recorder.events if e.event_type == EVENT_STATE_TRANSITION]
        assert transitions == []


# ---------------------------------------------------------------------------
# 3. Action success → action_completed (via SafetyActionExecutor + recorder)
# ---------------------------------------------------------------------------

class TestActionCompletedEvent:

    def test_action_completed_event_on_success(self) -> None:
        """Successful action emits action_completed via SafetyActionExecutor."""
        from src.safety_actions import SafetyActionExecutor

        class FakeControl:
            def stop_motion(self) -> None:
                pass

        recorder = SafetyEventRecorder()
        executor = SafetyActionExecutor(FakeControl(), recorder=recorder)
        executor._current_state = "link_lost"
        result = executor.execute("stop", reason="hb timeout")

        assert result.ok is True
        completed = [e for e in recorder.events if e.event_type == EVENT_ACTION_COMPLETED]
        assert len(completed) == 1
        assert completed[0].state == "link_lost"
        assert completed[0].actions is not None
        assert completed[0].actions[0]["action"] == "stop"


# ---------------------------------------------------------------------------
# 4. Action failure → action_failed
# ---------------------------------------------------------------------------

class TestActionFailedEvent:

    def test_action_failed_event_on_control_layer_exception(self) -> None:
        """A failing control layer emits action_failed event."""
        from src.safety_actions import SafetyActionExecutor

        class FailingControl:
            def stop_motion(self) -> None:
                raise RuntimeError("motor driver offline")

        recorder = SafetyEventRecorder()
        executor = SafetyActionExecutor(FailingControl(), recorder=recorder)
        executor._current_state = "kill"
        result = executor.execute("stop", reason="kill switch")

        assert result.ok is False
        failed = [e for e in recorder.events if e.event_type == EVENT_ACTION_FAILED]
        assert len(failed) == 1
        assert failed[0].error is not None
        assert "RuntimeError" in failed[0].error


# ---------------------------------------------------------------------------
# 5. once_key deduplication → action_skipped_once_key
# ---------------------------------------------------------------------------

class TestActionSkippedOnceKeyEvent:

    def test_action_skipped_once_key_event_on_dedup(self) -> None:
        """Second call with a consumed once_key emits action_skipped_once_key."""
        state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
        cfg = _cfg(link_lost_action="land")
        recorder = SafetyEventRecorder()
        executor = RecordingActionExecutor()
        supervisor = SafetySupervisor(
            lambda: state,
            cfg,
            MotionGate(),
            MemorySafetyLogger(),
            lambda reason: None,
            action_executor=executor,
            recorder=recorder,
        )
        decision = evaluate_safety(state, cfg, now=100.0)

        # First apply: action runs, once_key consumed.
        supervisor._apply_decision(decision, None)
        # Second apply with same state: action must be skipped.
        supervisor._apply_decision(decision, SafetyState.LINK_LOST)

        skipped = [e for e in recorder.events if e.event_type == EVENT_ACTION_SKIPPED_ONCE_KEY]
        assert len(skipped) == 1
        assert skipped[0].state == SafetyState.LINK_LOST.value
