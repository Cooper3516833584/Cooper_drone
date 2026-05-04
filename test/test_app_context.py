"""Tests for application context assembly."""

from __future__ import annotations

from pathlib import Path

from src.app_context import build_app_context
from src.safety_events import EVENT_STATE_TRANSITION, SafetyEventRecorder
from src.safety_policy import SafetyState


def test_app_context_close_is_idempotent(dry_run_config_path: Path) -> None:
    """Allow AppContext.close to be called more than once."""
    ctx = build_app_context(str(dry_run_config_path), dry_run=True)

    ctx.close()
    ctx.close()

    assert ctx.safety is None
    assert ctx.connection is None


def test_safety_supervisor_has_recorder_after_build(dry_run_config_path: Path) -> None:
    """build_app_context wires a SafetyEventRecorder into the supervisor."""
    ctx = build_app_context(str(dry_run_config_path), dry_run=True)

    assert ctx.safety is not None
    supervisor = ctx.safety
    assert supervisor._recorder is not None
    assert isinstance(supervisor._recorder, SafetyEventRecorder)


def test_safety_state_change_writes_to_recorder(dry_run_config_path: Path) -> None:
    """A safety state transition produces an event in the recorder."""
    from src.state import VehicleState
    from src.safety import evaluate_safety

    ctx = build_app_context(str(dry_run_config_path), dry_run=True)
    supervisor = ctx.safety
    assert supervisor is not None

    # Trigger KILL via the kill_switch_active flag (bypasses RC channel check).
    state = VehicleState(
        last_heartbeat_ts=100.0,
        mode="GUIDED",
        rc_last_update_ts=100.0,
        rc_channels={5: 1600, 7: 1000},
    )
    state.kill_switch_active = True
    decision = evaluate_safety(state, ctx.cfg.safety, now=100.0)

    assert decision.state is SafetyState.KILL
    supervisor._apply_decision(decision, None)

    recorder = supervisor._recorder
    assert recorder is not None
    transitions = [e for e in recorder.events if e.event_type == EVENT_STATE_TRANSITION]
    assert len(transitions) >= 1
    assert any(event.state == SafetyState.KILL.value for event in transitions)


def test_safety_supervisor_none_when_safety_disabled(dry_run_config_path: Path) -> None:
    """When safety is disabled, ctx.safety is None."""
    import yaml
    from pathlib import Path

    config_dir = dry_run_config_path.parent
    disabled_path = config_dir / "safety_disabled.yaml"
    data = yaml.safe_load(dry_run_config_path.read_text(encoding="utf-8"))
    data.setdefault("safety", {})["enabled"] = False
    disabled_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    ctx = build_app_context(str(disabled_path), dry_run=True)
    assert ctx.safety is None
