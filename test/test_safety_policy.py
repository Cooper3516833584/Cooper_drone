"""Tests for pure safety policy decisions."""

from __future__ import annotations

import inspect

from src.config_loader import SafetyConfig
from src.safety_policy import (
    KILL_CHANNEL,
    KILL_PWM_THRESHOLD,
    MODE_CHANNEL,
    MODE_GUIDED_PWM_THRESHOLD,
    SafetyState,
    evaluate_safety,
)
from src.state import VehicleState


def test_kill_priority_over_all_states() -> None:
    """KILL wins over revoke, link loss, and RC stale."""
    snapshot = VehicleState(
        last_heartbeat_ts=0.0,
        rc_last_update_ts=0.0,
        rc_channels={
            KILL_CHANNEL: KILL_PWM_THRESHOLD,
            MODE_CHANNEL: 1000,
        },
    )

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.KILL
    assert decision.should_cancel_mission is True
    assert decision.should_inhibit_motion is True


def test_revoke_priority_over_link_lost_and_rc_stale() -> None:
    """TAKEOVER_REVOKED wins over link loss and stale RC."""
    snapshot = VehicleState(
        last_heartbeat_ts=0.0,
        rc_last_update_ts=0.0,
        rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD - 1},
    )

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.TAKEOVER_REVOKED


def test_link_lost_priority_over_rc_stale() -> None:
    """LINK_LOST wins over stale RC when guided has not been revoked."""
    snapshot = VehicleState(
        last_heartbeat_ts=0.0,
        rc_last_update_ts=0.0,
        rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD},
    )

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.LINK_LOST


def test_rc_stale_applies_when_heartbeat_ok_and_revoke_not_triggered() -> None:
    """RC_STALE applies after heartbeat and revoke checks pass."""
    snapshot = VehicleState(
        last_heartbeat_ts=99.5,
        rc_last_update_ts=0.0,
        rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD},
    )

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.RC_STALE


def test_guided_pwm_returns_guided_allowed() -> None:
    """A guided PWM mode channel returns GUIDED_ALLOWED when inputs are fresh."""
    snapshot = VehicleState(
        last_heartbeat_ts=99.5,
        rc_last_update_ts=99.5,
        rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD},
    )

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.GUIDED_ALLOWED
    assert decision.actions == ()


def test_missing_rc_data_is_not_stale_by_itself() -> None:
    """Missing RC data does not create RC_STALE when no RC was ever seen."""
    snapshot = VehicleState(last_heartbeat_ts=99.5, mode=None, rc_last_update_ts=None, rc_channels={})

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg())

    assert decision.state is SafetyState.NORMAL


def test_link_lost_action_has_once_key() -> None:
    """LINK_LOST emits an action request with a once key on state entry."""
    snapshot = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)

    decision = evaluate_safety(snapshot=snapshot, now_monotonic=100.0, cfg=_cfg(link_lost_action="rtl"))

    assert decision.state is SafetyState.LINK_LOST
    assert len(decision.actions) == 1
    assert decision.actions[0].action == "rtl"
    assert decision.actions[0].once_key == "link_lost:rtl"


def test_state_specific_actions_are_independent() -> None:
    """Each unsafe state uses its own configured action."""
    cfg = _cfg(kill_action="land", revoke_action="loiter", link_lost_action="rtl", rc_stale_action="brake")
    kill = VehicleState(last_heartbeat_ts=99.5, rc_last_update_ts=99.5, rc_channels={KILL_CHANNEL: KILL_PWM_THRESHOLD})
    revoke = VehicleState(last_heartbeat_ts=99.5, rc_last_update_ts=99.5, rc_channels={MODE_CHANNEL: 1000})
    lost = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=99.5)
    stale = VehicleState(last_heartbeat_ts=99.5, mode="GUIDED", rc_last_update_ts=0.0)

    assert evaluate_safety(snapshot=kill, now_monotonic=100.0, cfg=cfg).actions[0].action == "land"
    assert evaluate_safety(snapshot=revoke, now_monotonic=100.0, cfg=cfg).actions[0].action == "loiter"
    assert evaluate_safety(snapshot=lost, now_monotonic=100.0, cfg=cfg).actions[0].action == "rtl"
    assert evaluate_safety(snapshot=stale, now_monotonic=100.0, cfg=cfg).actions[0].action == "brake"


def test_link_lost_once_action_can_retrigger_after_recovery() -> None:
    """Recovery to GUIDED_ALLOWED allows a later LINK_LOST once action to reappear."""
    recovered = VehicleState(last_heartbeat_ts=100.0, mode="GUIDED", rc_last_update_ts=100.0)
    lost_again = VehicleState(last_heartbeat_ts=0.0, mode="GUIDED", rc_last_update_ts=100.0)

    recovery_decision = evaluate_safety(
        snapshot=recovered,
        now_monotonic=100.0,
        cfg=_cfg(),
        previous_state=SafetyState.LINK_LOST,
    )
    lost_decision = evaluate_safety(
        snapshot=lost_again,
        now_monotonic=110.0,
        cfg=_cfg(),
        previous_state=recovery_decision.state,
    )

    assert recovery_decision.state is SafetyState.GUIDED_ALLOWED
    assert lost_decision.state is SafetyState.LINK_LOST
    assert len(lost_decision.actions) == 1
    assert lost_decision.actions[0].once_key == "link_lost:land"


def test_policy_does_not_use_threads_sleep_control_or_logging() -> None:
    """Keep policy free of side effects by source inspection."""
    import src.safety_policy as policy

    source = inspect.getsource(policy)

    forbidden = [
        "threading",
        "sleep",
        "Mavlink",
        "DroneMovement",
        "MotionGate",
        "cancel(",
        "logging",
        "logger",
        "control.",
        "movement.",
    ]
    for term in forbidden:
        assert term not in source


def _cfg(**overrides) -> SafetyConfig:
    values = {
        "enabled": True,
        "allow_arm": False,
        "require_guided": True,
        "require_heartbeat": True,
        "heartbeat_loss_timeout_s": 2.0,
        "rc_stale_timeout_s": 1.0,
        "manual_takeover_enabled": True,
        "kill_switch_enabled": True,
        "failsafe_action": "land",
    }
    values.update(overrides)
    return SafetyConfig(**values)
