"""Safety decision and supervisor framework."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from src.config_loader import SafetyConfig
from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionGate
from src.state import VehicleState


class SafetyState(Enum):
    """Safety state ordered by project priority."""

    NORMAL = "normal"
    GUIDED_ALLOWED = "guided_allowed"
    RC_STALE = "rc_stale"
    LINK_LOST = "link_lost"
    MANUAL_TAKEOVER = "manual_takeover"
    KILL = "kill"


@dataclass
class SafetyDecision:
    """Safety decision produced from vehicle state and safety config."""

    state: SafetyState
    reason: str
    cancel_mission: bool
    inhibit_motion: bool
    failsafe_action: str | None


def evaluate_safety(
    state: VehicleState,
    cfg: SafetyConfig,
    *,
    now: float,
    previous_state: SafetyState | None = None,
) -> SafetyDecision:
    """Evaluate vehicle safety using strict project priority order."""
    if not cfg.enabled:
        return SafetyDecision(SafetyState.NORMAL, "safety disabled", False, False, None)

    if cfg.kill_switch_enabled and _flag(state, "kill_switch_active", "kill"):
        return _unsafe_decision(SafetyState.KILL, "kill switch active", cfg, cancel=True)

    if cfg.manual_takeover_enabled and _manual_takeover_detected(state, cfg):
        return _unsafe_decision(SafetyState.MANUAL_TAKEOVER, "manual takeover detected", cfg, cancel=True)

    if cfg.require_heartbeat and _heartbeat_lost(state, cfg, now):
        return _unsafe_decision(SafetyState.LINK_LOST, "heartbeat link lost", cfg, cancel=True)

    if cfg.manual_takeover_enabled and _rc_stale(state, cfg, now):
        return _unsafe_decision(SafetyState.RC_STALE, "RC input is stale", cfg, cancel=True)

    if cfg.require_guided and _mode_is_guided(state.mode):
        return SafetyDecision(SafetyState.GUIDED_ALLOWED, "guided mode is active", False, False, None)

    return SafetyDecision(SafetyState.NORMAL, "safety checks passed", False, False, None)


class SafetySupervisor:
    """Background safety supervisor independent of mission execution."""

    def __init__(
        self,
        state_provider: Callable[[], VehicleState],
        cfg: SafetyConfig,
        motion_gate: MotionGate,
        logger: RuntimeLogger,
        cancel_mission: Callable[[str], None],
        *,
        check_interval_s: float = 0.05,
    ) -> None:
        """Create a safety supervisor."""
        self._state_provider = state_provider
        self._cfg = cfg
        self._motion_gate = motion_gate
        self._logger = logger
        self._cancel_mission = cancel_mission
        self._check_interval_s = check_interval_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._current_state = SafetyState.NORMAL
        self._inhibit_source_state: SafetyState | None = None

    def start(self) -> None:
        """Start the background safety supervisor."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="safety-supervisor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background safety supervisor."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self._logger.app_logger.error("Safety supervisor thread did not stop before timeout")
        self._thread = None

    def current_state(self) -> SafetyState:
        """Return the current supervisor safety state."""
        with self._lock:
            return self._current_state

    def _run(self) -> None:
        previous_state: SafetyState | None = None
        while not self._stop_event.is_set():
            try:
                decision = evaluate_safety(
                    self._state_provider(),
                    self._cfg,
                    now=time.time(),
                    previous_state=previous_state,
                )
                self._apply_decision(decision, previous_state)
                previous_state = decision.state
            except Exception:
                self._logger.app_logger.exception("Safety supervisor check failed")
            self._stop_event.wait(self._check_interval_s)

    def _apply_decision(self, decision: SafetyDecision, previous_state: SafetyState | None) -> None:
        state_changed = decision.state != previous_state
        with self._lock:
            self._current_state = decision.state

        if state_changed:
            self._logger.safety(
                "safety_state_changed",
                state=decision.state.value,
                reason=decision.reason,
                cancel_mission=decision.cancel_mission,
                inhibit_motion=decision.inhibit_motion,
                failsafe_action=decision.failsafe_action,
            )

        if decision.cancel_mission and state_changed:
            self._cancel_mission(decision.reason)

        if decision.inhibit_motion:
            self._motion_gate.inhibit(decision.reason)
            self._inhibit_source_state = decision.state
        elif self._can_auto_clear_gate():
            self._motion_gate.clear()
            self._inhibit_source_state = None

        if decision.failsafe_action and state_changed:
            self._logger.safety(
                "failsafe_action_recorded",
                state=decision.state.value,
                action=decision.failsafe_action,
            )

    def _can_auto_clear_gate(self) -> bool:
        if self._inhibit_source_state is None:
            return False
        return self._inhibit_source_state not in {SafetyState.KILL, SafetyState.MANUAL_TAKEOVER}


def _unsafe_decision(
    state: SafetyState,
    reason: str,
    cfg: SafetyConfig,
    *,
    cancel: bool,
) -> SafetyDecision:
    return SafetyDecision(
        state=state,
        reason=reason,
        cancel_mission=cancel,
        inhibit_motion=True,
        failsafe_action=_failsafe_action(cfg),
    )


def _failsafe_action(cfg: SafetyConfig) -> str | None:
    if cfg.failsafe_action == "none":
        return None
    return cfg.failsafe_action


def _flag(state: VehicleState, *names: str) -> bool:
    return any(bool(getattr(state, name, False)) for name in names)


def _manual_takeover_detected(state: VehicleState, cfg: SafetyConfig) -> bool:
    if _flag(state, "manual_takeover", "manual_takeover_active"):
        return True
    if not cfg.require_guided or state.mode is None:
        return False
    return not _mode_is_guided(state.mode)


def _heartbeat_lost(state: VehicleState, cfg: SafetyConfig, now: float) -> bool:
    if state.last_heartbeat_ts is None:
        return True
    return now - state.last_heartbeat_ts > cfg.heartbeat_loss_timeout_s


def _rc_stale(state: VehicleState, cfg: SafetyConfig, now: float) -> bool:
    if state.rc_last_update_ts is None:
        return bool(state.rc_channels)
    return now - state.rc_last_update_ts > cfg.rc_stale_timeout_s


def _mode_is_guided(mode: str | None) -> bool:
    if mode is None:
        return False
    return mode.upper() in {"GUIDED", "GUIDED_NOGPS"}
