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


# ---------------------------------------------------------------------------
# Additional Task-09 Layer-4 cases
# ---------------------------------------------------------------------------


def test_once_key_skip_is_recorded_as_event() -> None:
    """action_skipped_once_key event is emitted when a once-key is deduped."""
    from src.safety_events import EVENT_ACTION_SKIPPED_ONCE_KEY, SafetyEventRecorder

    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
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
        check_interval_s=0.01,
    )
    decision = evaluate_safety(state, cfg, now=100.0)

    supervisor._apply_decision(decision, None)
    # Second call with same state → once-key already consumed.
    supervisor._apply_decision(decision, SafetyState.LINK_LOST)

    skipped = [e for e in recorder.events if e.event_type == EVENT_ACTION_SKIPPED_ONCE_KEY]
    assert len(skipped) == 1


def test_motion_inhibited_event_is_emitted() -> None:
    """motion_inhibited event is emitted whenever inhibit_motion fires."""
    from src.safety_events import EVENT_MOTION_INHIBITED, SafetyEventRecorder

    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
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

    inhibited = [e for e in recorder.events if e.event_type == EVENT_MOTION_INHIBITED]
    assert len(inhibited) >= 1


def test_mission_cancelled_event_is_emitted_on_state_change() -> None:
    """mission_cancelled event is emitted when cancel_mission fires."""
    from src.safety_events import EVENT_MISSION_CANCELLED, SafetyEventRecorder

    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
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
    supervisor._apply_decision(decision, None)  # state change → cancel fires

    cancelled = [e for e in recorder.events if e.event_type == EVENT_MISSION_CANCELLED]
    assert len(cancelled) == 1


def test_supervisor_error_event_emitted_on_loop_exception() -> None:
    """supervisor_error event is emitted when the safety loop raises."""
    import time

    from src.safety_events import EVENT_SUPERVISOR_ERROR, SafetyEventRecorder

    call_count = 0

    def bad_provider() -> VehicleState:
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            raise RuntimeError("state fetch failed")
        return VehicleState(
            last_heartbeat_ts=time.time(),
            mode="GUIDED",
            rc_last_update_ts=time.time(),
        )

    cfg = _safety_config()
    recorder = SafetyEventRecorder()
    supervisor = SafetySupervisor(
        bad_provider,
        cfg,
        MotionGate(),
        MemorySafetyLogger(),
        lambda reason: None,
        recorder=recorder,
        check_interval_s=0.01,
    )
    supervisor.start()
    # Give the loop time to execute the bad call and then recover.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        errors = [e for e in recorder.events if e.event_type == EVENT_SUPERVISOR_ERROR]
        if errors:
            break
        time.sleep(0.01)
    supervisor.stop()

    errors = [e for e in recorder.events if e.event_type == EVENT_SUPERVISOR_ERROR]
    assert len(errors) >= 1
    assert errors[0].error is not None


def test_link_lost_inhibits_motion_gate() -> None:
    """LINK_LOST decision inhibits the MotionGate."""
    gate = MotionGate()
    state = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config(link_lost_action="land")
    supervisor = _supervisor(lambda: state, cfg, gate, list().append, RecordingActionExecutor())

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    assert gate.is_inhibited() is True


def test_recovery_to_guided_allowed_clears_motion_gate() -> None:
    """Recovering to GUIDED_ALLOWED clears the MotionGate when supervisor inhibited it."""
    gate = MotionGate()
    lost = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    recovered = VehicleState(last_heartbeat_ts=100.0, mode="GUIDED", rc_last_update_ts=100.0)
    cfg = _safety_config(link_lost_action="land")
    supervisor = _supervisor(lambda: lost, cfg, gate, list().append, RecordingActionExecutor())

    lost_decision = evaluate_safety(lost, cfg, now=100.0)
    recovered_decision = evaluate_safety(
        recovered, cfg, now=100.0, previous_state=SafetyState.LINK_LOST,
    )

    supervisor._apply_decision(lost_decision, None)
    assert gate.is_inhibited() is True

    supervisor._apply_decision(recovered_decision, SafetyState.LINK_LOST)
    assert gate.is_inhibited() is False


def test_recovery_only_clears_own_inhibit() -> None:
    """MotionGate is not cleared if the supervisor did not set the inhibit."""
    gate = MotionGate()
    gate.inhibit("external module")
    state = VehicleState(last_heartbeat_ts=99.5, mode="GUIDED", rc_last_update_ts=99.5)
    cfg = _safety_config()
    supervisor = _supervisor(lambda: state, cfg, gate, list().append, RecordingActionExecutor())

    supervisor._apply_decision(evaluate_safety(state, cfg, now=100.0), None)

    # Gate is still inhibited because the supervisor didn't own the inhibit.
    assert gate.is_inhibited() is True
    assert gate.reason() == "external module"


def test_recovery_does_not_clear_external_inhibit_after_safety_inhibit() -> None:
    """MotionGate is not cleared if another owner overwrote the safety reason."""
    gate = MotionGate()
    lost = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    recovered = VehicleState(last_heartbeat_ts=100.0, mode="GUIDED", rc_last_update_ts=100.0)
    cfg = _safety_config(link_lost_action="land")
    supervisor = _supervisor(lambda: lost, cfg, gate, list().append, RecordingActionExecutor())

    lost_decision = evaluate_safety(lost, cfg, now=100.0)
    recovered_decision = evaluate_safety(
        recovered, cfg, now=100.0, previous_state=SafetyState.LINK_LOST,
    )
    supervisor._apply_decision(lost_decision, None)
    gate.inhibit("external module")
    supervisor._apply_decision(recovered_decision, SafetyState.LINK_LOST)

    assert gate.is_inhibited() is True
    assert gate.reason() == "external module"


def test_motion_gate_cleared_logs_event() -> None:
    """Clearing the MotionGate on recovery writes a safety log record."""
    logger = MemorySafetyLogger()
    gate = MotionGate()
    lost = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    recovered = VehicleState(last_heartbeat_ts=100.0, mode="GUIDED", rc_last_update_ts=100.0)
    cfg = _safety_config(link_lost_action="land")
    supervisor = SafetySupervisor(
        lambda: lost, cfg, gate, logger, lambda reason: None,
        action_executor=RecordingActionExecutor(), check_interval_s=0.01,
    )

    lost_decision = evaluate_safety(lost, cfg, now=100.0)
    recovered_decision = evaluate_safety(
        recovered, cfg, now=100.0, previous_state=SafetyState.LINK_LOST,
    )
    supervisor._apply_decision(lost_decision, None)
    supervisor._apply_decision(recovered_decision, SafetyState.LINK_LOST)

    cleared = [r for r in logger.safety_records if r["name"] == "motion_gate_cleared"]
    assert len(cleared) == 1
    assert cleared[0]["previous_state"] == SafetyState.LINK_LOST.value


def test_recovery_after_link_lost_allows_second_action() -> None:
    """After recovery to GUIDED_ALLOWED, a second LINK_LOST executes its action again."""
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

    assert len(executor.calls) == 2
    assert executor.calls[0] == ("land", "heartbeat link lost")
    assert executor.calls[1] == ("land", "heartbeat link lost")
