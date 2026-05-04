"""Tests for safety action execution."""

from __future__ import annotations

from src.config_loader import SafetyConfig
from src.motion_gate import MotionGate
from src.safety import SafetySupervisor, evaluate_safety
from src.safety_actions import SafetyActionExecutor
from src.state import VehicleState


def test_stop_calls_stop_motion() -> None:
    """STOP sends zero motion through the control layer."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).stop_motion(reason="link_lost")

    assert result.ok is True
    assert control.calls == [("stop_motion",)]


def test_land_calls_land_nowait_when_available() -> None:
    """LAND uses non-blocking land when the control layer provides it."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).land(reason="link_lost")

    assert result.ok is True
    assert control.calls == [("land_nowait",)]


def test_loiter_calls_set_mode_nowait() -> None:
    """LOITER maps to a mode change when no direct helper exists."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).loiter(reason="takeover")

    assert result.ok is True
    assert control.calls == [("set_mode_nowait", "LOITER")]


def test_brake_calls_set_mode_nowait() -> None:
    """BRAKE maps to a mode change when no direct helper exists."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).brake(reason="kill")

    assert result.ok is True
    assert control.calls == [("set_mode_nowait", "BRAKE")]


def test_action_exception_returns_failed_result() -> None:
    """Executor captures action exceptions instead of propagating them."""
    control = FakeControl(fail_on="stop_motion")
    result = SafetyActionExecutor(control, MemorySafetyLogger()).execute("stop", reason="link_lost")

    assert result.ok is False
    assert "RuntimeError" in (result.error or "")


def test_disarm_and_force_disarm_are_not_default_failsafe_actions() -> None:
    """Disarm actions require explicit executor opt-in."""
    control = FakeControl()
    logger = MemorySafetyLogger()
    executor = SafetyActionExecutor(control, logger)

    disarm = executor.execute("disarm", reason="kill")
    force_disarm = executor.execute("force_disarm", reason="kill")

    assert disarm.ok is False
    assert force_disarm.ok is False
    assert control.calls == []
    assert logger.critical_messages == ["Force disarm safety action requested: kill"]


def test_safety_supervisor_executes_policy_actions() -> None:
    """SafetySupervisor delegates policy actions to SafetyActionExecutor."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=100.0)
    cfg = _safety_config(link_lost_action="brake")
    gate = MotionGate()
    logger = MemorySafetyLogger()
    control = FakeControl()
    supervisor = SafetySupervisor(
        lambda: state,
        cfg,
        gate,
        logger,
        lambda reason: None,
        action_executor=SafetyActionExecutor(control, logger),
    )
    decision = evaluate_safety(state, cfg, now=100.0)

    supervisor._apply_decision(decision, None)

    assert control.calls == [("set_mode_nowait", "BRAKE")]
    assert logger.safety_records[-1]["name"] == "safety_action_result"
    assert logger.safety_records[-1]["ok"] is True


class FakeControl:
    """Fake low-level control surface used by safety action tests."""

    def __init__(self, *, fail_on: str | None = None) -> None:
        self.calls: list[tuple] = []
        self.fail_on = fail_on

    def stop_motion(self) -> None:
        self._capture("stop_motion")

    def land_nowait(self) -> None:
        self._capture("land_nowait")

    def set_mode_nowait(self, mode: str) -> None:
        self._capture("set_mode_nowait", mode)

    def disarm(self) -> None:
        self._capture("disarm")

    def force_disarm(self) -> None:
        self._capture("force_disarm")

    def _capture(self, *call: str) -> None:
        if self.fail_on == call[0]:
            raise RuntimeError(f"{call[0]} failed")
        self.calls.append(tuple(call))


class MemorySafetyLogger:
    """Capture structured safety logs without touching files."""

    def __init__(self) -> None:
        self.safety_records: list[dict] = []
        self.app_logger = self
        self.critical_messages: list[str] = []

    def safety(self, name: str, **fields) -> None:
        self.safety_records.append({"name": name, **fields})

    def critical(self, message: str, *args) -> None:
        if args:
            message = message % args
        self.critical_messages.append(message)

    def error(self, message: str, *args) -> None:
        pass

    def exception(self, message: str, *args) -> None:
        pass


def _safety_config(**overrides) -> SafetyConfig:
    values = {
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


# ---------------------------------------------------------------------------
# Additional Task-09 Layer-3 cases
# ---------------------------------------------------------------------------


def test_action_result_contains_action_reason_fields() -> None:
    """SafetyActionResult carries action, reason, ok, and error fields."""
    from src.safety_actions import SafetyActionResult

    result = SafetyActionResult(
        action="land",
        reason="link lost",
        ok=True,
        error=None,
    )
    assert result.action == "land"
    assert result.reason == "link lost"
    assert result.ok is True
    assert result.error is None


def test_failed_action_result_contains_error_field() -> None:
    """A failed action result carries a non-None error field."""
    control = FakeControl(fail_on="stop_motion")
    result = SafetyActionExecutor(control, MemorySafetyLogger()).execute("stop", reason="kill")
    assert result.ok is False
    assert result.error is not None
    assert result.action == "stop"
    assert result.reason == "kill"


def test_force_disarm_requires_explicit_allow_force_disarm_flag() -> None:
    """force_disarm must be gated behind allow_force_disarm=True."""
    control = FakeControl()
    # Without opt-in, force_disarm must be rejected.
    executor_no = SafetyActionExecutor(control, MemorySafetyLogger(), allow_force_disarm=False)
    result_no = executor_no.execute("force_disarm", reason="kill")
    assert result_no.ok is False
    assert "not enabled" in (result_no.error or "").lower()

    # With opt-in, call should proceed.
    executor_yes = SafetyActionExecutor(control, MemorySafetyLogger(), allow_force_disarm=True)
    result_yes = executor_yes.execute("force_disarm", reason="kill")
    assert result_yes.ok is True


def test_disarm_requires_explicit_allow_disarm_flag() -> None:
    """disarm must be gated behind allow_disarm=True."""
    control = FakeControl()
    executor = SafetyActionExecutor(control, MemorySafetyLogger(), allow_disarm=False)
    result = executor.execute("disarm", reason="kill")
    assert result.ok is False
    assert control.calls == []


def test_unsupported_action_name_returns_failed_result() -> None:
    """An unrecognised action name returns ok=False with an error message."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).execute(
        "self_destruct", reason="unknown"
    )
    assert result.ok is False
    assert result.error is not None


def test_rtl_falls_back_to_set_mode_when_no_rtl_method() -> None:
    """RTL action falls back to set_mode_nowait when rtl() is absent."""
    control = FakeControl()
    result = SafetyActionExecutor(control, MemorySafetyLogger()).rtl(reason="link_lost")
    assert result.ok is True
    assert ("set_mode_nowait", "RTL") in control.calls


def test_executor_recorder_integration() -> None:
    """SafetyActionExecutor emits structured events via recorder when provided."""
    from src.safety_events import EVENT_ACTION_COMPLETED, SafetyEventRecorder

    control = FakeControl()
    recorder = SafetyEventRecorder()
    executor = SafetyActionExecutor(control, MemorySafetyLogger(), recorder=recorder)
    executor._current_state = "link_lost"
    result = executor.execute("stop", reason="hb timeout")

    assert result.ok is True
    completed = [e for e in recorder.events if e.event_type == EVENT_ACTION_COMPLETED]
    assert len(completed) == 1
    assert completed[0].state == "link_lost"


def test_executor_with_allow_force_disarm_executes_force_disarm() -> None:
    """When allow_force_disarm=True, the executor calls force_disarm on the session."""
    control = FakeControl()
    executor = SafetyActionExecutor(
        control, MemorySafetyLogger(), allow_force_disarm=True,
    )
    result = executor.execute("force_disarm", reason="kill")

    assert result.ok is True
    assert ("force_disarm",) in control.calls


def test_force_disarm_without_control_support_fails_even_when_allowed() -> None:
    """Allowed force_disarm still requires an explicit force_disarm control method."""
    class NoForceDisarmControl:
        def disarm(self) -> None:
            pass

    result = SafetyActionExecutor(
        NoForceDisarmControl(),
        MemorySafetyLogger(),
        allow_force_disarm=True,
    ).execute("force_disarm", reason="kill")

    assert result.ok is False
    assert "not supported" in (result.error or "")
