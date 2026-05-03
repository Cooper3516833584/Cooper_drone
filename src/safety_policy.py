"""Pure safety policy decisions with no threads or side effects."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from src.config_loader import SafetyConfig
from src.state import VehicleState


MODE_CHANNEL = 5
MODE_GUIDED_PWM_THRESHOLD = 1500
KILL_CHANNEL = 7
KILL_PWM_THRESHOLD = 1800


class SafetyState(Enum):
    """Safety state ordered by project priority."""

    NORMAL = "normal"
    GUIDED_ALLOWED = "guided_allowed"
    RC_STALE = "rc_stale"
    LINK_LOST = "link_lost"
    TAKEOVER_REVOKED = "takeover_revoked"
    MANUAL_TAKEOVER = "takeover_revoked"
    KILL = "kill"


@dataclass(frozen=True)
class SafetyActionRequest:
    """A requested safety action emitted by pure policy."""

    action: str
    reason: str
    once_key: str | None = None
    allow_retry: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    """Pure safety decision returned by policy evaluation."""

    state: SafetyState
    reason: str
    should_cancel_mission: bool
    should_inhibit_motion: bool
    actions: tuple[SafetyActionRequest, ...] = ()

    @property
    def cancel_mission(self) -> bool:
        """Return the legacy cancel flag."""
        return self.should_cancel_mission

    @property
    def inhibit_motion(self) -> bool:
        """Return the legacy inhibit flag."""
        return self.should_inhibit_motion

    @property
    def failsafe_action(self) -> str | None:
        """Return the first requested action for legacy callers."""
        if not self.actions:
            return None
        return self.actions[0].action


def evaluate_safety(
    *,
    snapshot: VehicleState,
    now_monotonic: float,
    cfg: SafetyConfig,
    previous_state: SafetyState | None = None,
    heartbeat_age_s: float | None = None,
) -> SafetyDecision:
    """Evaluate safety policy without side effects."""
    if not cfg.enabled:
        return SafetyDecision(SafetyState.NORMAL, "safety disabled", False, False)

    if cfg.kill_switch_enabled and _kill_active(snapshot):
        return _unsafe_decision(
            SafetyState.KILL,
            "kill channel active",
            cfg,
            previous_state,
            action=_action_for_state(cfg, SafetyState.KILL),
            allow_retry=False,
        )

    if cfg.manual_takeover_enabled and _takeover_revoked(snapshot, cfg):
        return _unsafe_decision(
            SafetyState.TAKEOVER_REVOKED,
            "guided control revoked",
            cfg,
            previous_state,
            action=_action_for_state(cfg, SafetyState.TAKEOVER_REVOKED),
            allow_retry=False,
        )

    if cfg.require_heartbeat and _link_lost(snapshot, cfg, now_monotonic, heartbeat_age_s):
        return _unsafe_decision(
            SafetyState.LINK_LOST,
            "heartbeat link lost",
            cfg,
            previous_state,
            action=_action_for_state(cfg, SafetyState.LINK_LOST),
            allow_retry=False,
        )

    if cfg.manual_takeover_enabled and _rc_stale(snapshot, cfg, now_monotonic):
        return _unsafe_decision(
            SafetyState.RC_STALE,
            "RC input is stale",
            cfg,
            previous_state,
            action=_action_for_state(cfg, SafetyState.RC_STALE),
            allow_retry=False,
        )

    if cfg.require_guided and _guided_allowed(snapshot):
        return SafetyDecision(SafetyState.GUIDED_ALLOWED, "guided control allowed", False, False)

    return SafetyDecision(SafetyState.NORMAL, "safety checks passed", False, False)


def _unsafe_decision(
    state: SafetyState,
    reason: str,
    cfg: SafetyConfig,
    previous_state: SafetyState | None,
    *,
    action: str | None,
    allow_retry: bool,
) -> SafetyDecision:
    actions: tuple[SafetyActionRequest, ...] = ()
    if action is not None and previous_state is not state:
        actions = (
            SafetyActionRequest(
                action=action,
                reason=reason,
                once_key=f"{state.value}:{action}",
                allow_retry=allow_retry,
            ),
        )

    return SafetyDecision(
        state=state,
        reason=reason,
        should_cancel_mission=True,
        should_inhibit_motion=True,
        actions=actions,
    )


def _action_for_state(cfg: SafetyConfig, state: SafetyState) -> str | None:
    action = {
        SafetyState.KILL: getattr(cfg, "kill_action", cfg.failsafe_action),
        SafetyState.TAKEOVER_REVOKED: getattr(cfg, "revoke_action", cfg.failsafe_action),
        SafetyState.MANUAL_TAKEOVER: getattr(cfg, "revoke_action", cfg.failsafe_action),
        SafetyState.LINK_LOST: getattr(cfg, "link_lost_action", cfg.failsafe_action),
        SafetyState.RC_STALE: getattr(cfg, "rc_stale_action", cfg.failsafe_action),
    }.get(state, cfg.failsafe_action)
    action = action.lower()
    if action == "none":
        return None
    return action


def _kill_active(snapshot: VehicleState) -> bool:
    if bool(getattr(snapshot, "kill_switch_active", False)) or bool(getattr(snapshot, "kill", False)):
        return True
    pwm = snapshot.rc_channels.get(KILL_CHANNEL)
    return pwm is not None and pwm >= KILL_PWM_THRESHOLD


def _takeover_revoked(snapshot: VehicleState, cfg: SafetyConfig) -> bool:
    if bool(getattr(snapshot, "manual_takeover", False)) or bool(getattr(snapshot, "manual_takeover_active", False)):
        return True
    if bool(getattr(snapshot, "takeover_revoked", False)):
        return True

    mode_pwm = snapshot.rc_channels.get(MODE_CHANNEL)
    if mode_pwm is not None:
        return mode_pwm < MODE_GUIDED_PWM_THRESHOLD

    if not cfg.require_guided or snapshot.mode is None:
        return False
    return not _mode_is_guided(snapshot.mode)


def _link_lost(
    snapshot: VehicleState,
    cfg: SafetyConfig,
    now_monotonic: float,
    heartbeat_age_s: float | None,
) -> bool:
    if heartbeat_age_s is not None:
        return heartbeat_age_s > cfg.heartbeat_loss_timeout_s
    if snapshot.last_heartbeat_ts is None:
        return True
    return now_monotonic - snapshot.last_heartbeat_ts > cfg.heartbeat_loss_timeout_s


def _rc_stale(snapshot: VehicleState, cfg: SafetyConfig, now_monotonic: float) -> bool:
    if snapshot.rc_last_update_ts is None:
        return bool(snapshot.rc_channels)
    return now_monotonic - snapshot.rc_last_update_ts > cfg.rc_stale_timeout_s


def _guided_allowed(snapshot: VehicleState) -> bool:
    mode_pwm = snapshot.rc_channels.get(MODE_CHANNEL)
    if mode_pwm is not None:
        return mode_pwm >= MODE_GUIDED_PWM_THRESHOLD
    return _mode_is_guided(snapshot.mode)


def _mode_is_guided(mode: str | None) -> bool:
    if mode is None:
        return False
    return mode.upper() in {"GUIDED", "GUIDED_NOGPS"}
