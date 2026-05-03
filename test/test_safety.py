"""Tests for safety decisions and supervisor."""

from __future__ import annotations

import logging
import time
from pathlib import Path

from src.config_loader import SafetyConfig
from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionGate
from src.safety import SafetyState, SafetySupervisor, evaluate_safety
from src.safety_policy import MODE_CHANNEL, MODE_GUIDED_PWM_THRESHOLD
from src.state import VehicleState


def test_kill_priority_over_all_states() -> None:
    """KILL has the highest safety priority."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="STABILIZE", rc_last_update_ts=0.0)
    state.kill_switch_active = True

    decision = evaluate_safety(state, _safety_config(), now=100.0)

    assert decision.state is SafetyState.KILL
    assert decision.cancel_mission is True
    assert decision.inhibit_motion is True


def test_manual_takeover_priority_over_link_lost() -> None:
    """Manual takeover is evaluated before link lost."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="STABILIZE")

    decision = evaluate_safety(state, _safety_config(), now=100.0)

    assert decision.state is SafetyState.MANUAL_TAKEOVER


def test_link_lost_cancels_and_inhibits_motion() -> None:
    """Link loss cancels the mission and blocks motion."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED")

    decision = evaluate_safety(state, _safety_config(), now=100.0)

    assert decision.state is SafetyState.LINK_LOST
    assert decision.cancel_mission is True
    assert decision.inhibit_motion is True


def test_rc_stale_uses_config_decision() -> None:
    """RC stale is detected only when manual takeover monitoring is enabled."""
    state = VehicleState(last_heartbeat_ts=99.5, mode="GUIDED", rc_last_update_ts=0.0)

    enabled_decision = evaluate_safety(state, _safety_config(), now=100.0)
    disabled_decision = evaluate_safety(
        state,
        _safety_config(manual_takeover_enabled=False),
        now=100.0,
    )

    assert enabled_decision.state is SafetyState.RC_STALE
    assert disabled_decision.state is SafetyState.GUIDED_ALLOWED


def test_guided_allowed_state_is_recognized() -> None:
    """Recognize guided mode as allowed when checks pass."""
    state = VehicleState(last_heartbeat_ts=99.5, mode="GUIDED", rc_last_update_ts=99.5)

    decision = evaluate_safety(state, _safety_config(), now=100.0)

    assert decision.state is SafetyState.GUIDED_ALLOWED
    assert decision.cancel_mission is False
    assert decision.inhibit_motion is False


def test_safety_supervisor_start_stop_no_thread_leak() -> None:
    """Start and stop the safety supervisor cleanly."""
    state = VehicleState(last_heartbeat_ts=time.time(), mode="GUIDED", rc_last_update_ts=time.time())
    gate = MotionGate()
    cancelled: list[str] = []
    supervisor = SafetySupervisor(
        lambda: state,
        _safety_config(),
        gate,
        _fake_runtime_logger(),
        cancelled.append,
        check_interval_s=0.01,
    )

    supervisor.start()
    time.sleep(0.05)
    supervisor.stop()

    assert supervisor.current_state() is SafetyState.GUIDED_ALLOWED
    assert cancelled == []


def test_kill_triggers_cancel_inhibit_and_executor_action() -> None:
    """KILL state cancels the mission, inhibits motion, and executes its action."""
    state = VehicleState(last_heartbeat_ts=99.5, rc_last_update_ts=99.5)
    state.kill_switch_active = True
    cfg = _safety_config(kill_action="land")
    gate = MotionGate()
    cancelled: list[str] = []
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: state, cfg, gate, cancelled.append, executor)

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    assert cancelled == ["kill channel active"]
    assert gate.is_inhibited() is True
    assert executor.calls == [("land", "kill channel active")]


def test_takeover_revoked_triggers_revoke_action() -> None:
    """TAKEOVER_REVOKED uses the configured revoke action."""
    state = VehicleState(last_heartbeat_ts=99.5, mode="STABILIZE", rc_last_update_ts=99.5)
    cfg = _safety_config(revoke_action="loiter")
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: state, cfg, MotionGate(), list().append, executor)

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    assert executor.calls == [("loiter", "guided control revoked")]


def test_link_lost_triggers_link_lost_action() -> None:
    """LINK_LOST uses the configured link-loss action."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: state, cfg, MotionGate(), list().append, executor)

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    assert executor.calls == [("land", "heartbeat link lost")]


def test_link_lost_continuous_state_executes_once_action_once() -> None:
    """LINK_LOST once action is not repeated while the state is continuous."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: state, cfg, MotionGate(), list().append, executor)
    decision = evaluate_safety(state, cfg, now=100.0)

    supervisor._apply_decision(decision, None)
    supervisor._apply_decision(decision, SafetyState.LINK_LOST)

    assert executor.calls == [("land", "heartbeat link lost")]


def test_link_lost_action_retriggers_after_safe_recovery() -> None:
    """A safe state clears once-action memory for later link loss."""
    lost = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    recovered = VehicleState(last_heartbeat_ts=100.0, mode="GUIDED", rc_last_update_ts=100.0)
    cfg = _safety_config(link_lost_action="land")
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: lost, cfg, MotionGate(), list().append, executor)

    lost_decision = evaluate_safety(lost, cfg, now=100.0)
    recovered_decision = evaluate_safety(recovered, cfg, now=100.0, previous_state=SafetyState.LINK_LOST)
    lost_again_decision = evaluate_safety(lost, cfg, now=110.0, previous_state=recovered_decision.state)
    supervisor._apply_decision(lost_decision, None)
    supervisor._apply_decision(recovered_decision, SafetyState.LINK_LOST)
    supervisor._apply_decision(lost_again_decision, recovered_decision.state)

    assert executor.calls == [
        ("land", "heartbeat link lost"),
        ("land", "heartbeat link lost"),
    ]


def test_rc_stale_triggers_own_action() -> None:
    """RC_STALE uses rc_stale_action rather than the revoke action."""
    state = VehicleState(
        last_heartbeat_ts=99.5,
        rc_last_update_ts=0.0,
        rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD},
    )
    cfg = _safety_config(revoke_action="loiter", rc_stale_action="brake")
    executor = RecordingActionExecutor()
    supervisor = _supervisor(lambda: state, cfg, MotionGate(), list().append, executor)

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    assert executor.calls == [("brake", "RC input is stale")]


def test_action_executor_exception_does_not_stop_supervisor_thread() -> None:
    """Executor failures are logged and do not kill the safety loop."""
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
    logger = MemorySafetyLogger()
    supervisor = SafetySupervisor(
        lambda: state,
        cfg,
        MotionGate(),
        logger,
        lambda reason: None,
        action_executor=RecordingActionExecutor(raise_on_execute=True),
        check_interval_s=0.01,
    )

    supervisor.start()
    time.sleep(0.05)

    assert supervisor._thread is not None
    assert supervisor._thread.is_alive()
    assert any(record["name"] == "safety_action_executor_failed" for record in logger.safety_records)
    supervisor.stop()


def test_stop_reliably_ends_thread() -> None:
    """stop() joins the supervisor thread."""
    state = VehicleState(last_heartbeat_ts=time.time(), mode="GUIDED", rc_last_update_ts=time.time())
    supervisor = _supervisor(lambda: state, _safety_config(), MotionGate(), list().append, RecordingActionExecutor())

    supervisor.start()
    supervisor.stop()

    assert supervisor._thread is None


def test_repeated_start_does_not_create_multiple_threads() -> None:
    """Calling start twice keeps one running safety thread."""
    state = VehicleState(last_heartbeat_ts=time.time(), mode="GUIDED", rc_last_update_ts=time.time())
    supervisor = _supervisor(lambda: state, _safety_config(), MotionGate(), list().append, RecordingActionExecutor())

    supervisor.start()
    first_thread = supervisor._thread
    supervisor.start()

    assert supervisor._thread is first_thread
    supervisor.stop()


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


def _fake_runtime_logger() -> RuntimeLogger:
    logger = logging.getLogger("test.safety")
    logger.addHandler(logging.NullHandler())
    return RuntimeLogger(run_id="test-run", run_dir=Path("logs/test-safety"), app_logger=logger)


def _supervisor(
    state_provider,
    cfg: SafetyConfig,
    gate: MotionGate,
    cancel_mission,
    executor,
) -> SafetySupervisor:
    return SafetySupervisor(
        state_provider,
        cfg,
        gate,
        MemorySafetyLogger(),
        cancel_mission,
        action_executor=executor,
        check_interval_s=0.01,
    )


class RecordingActionExecutor:
    """Fake safety action executor for supervisor tests."""

    def __init__(self, *, raise_on_execute: bool = False) -> None:
        self.calls: list[tuple[str, str]] = []
        self.raise_on_execute = raise_on_execute

    def execute(self, action: str, *, reason: str) -> None:
        self.calls.append((action, reason))
        if self.raise_on_execute:
            raise RuntimeError("executor failed")


class MemorySafetyLogger:
    """Small logger stub with structured safety capture."""

    def __init__(self) -> None:
        self.safety_records: list[dict] = []
        self.app_logger = self
        self.exceptions: list[str] = []
        self.errors: list[str] = []

    def safety(self, name: str, **fields) -> None:
        self.safety_records.append({"name": name, **fields})

    def exception(self, message: str, *args) -> None:
        self.exceptions.append(message % args if args else message)

    def error(self, message: str, *args) -> None:
        self.errors.append(message % args if args else message)
