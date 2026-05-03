"""Tests for safety decisions and supervisor."""

from __future__ import annotations

import logging
import time
from pathlib import Path

from src.config_loader import SafetyConfig
from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionGate
from src.safety import SafetyState, SafetySupervisor, evaluate_safety
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
