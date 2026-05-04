"""Safety decision and supervisor framework."""

from __future__ import annotations

import threading
import time
from typing import Callable

from src.config_loader import SafetyConfig
from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionGate
from src.safety_actions import SafetyActionExecutor
from src.safety_events import SafetyEventRecorder
from src.safety_policy import SafetyDecision, SafetyState
from src.safety_policy import evaluate_safety as evaluate_safety_policy
from src.state import VehicleState


def evaluate_safety(
    state: VehicleState,
    cfg: SafetyConfig,
    *,
    now: float,
    previous_state: SafetyState | None = None,
) -> SafetyDecision:
    """Evaluate vehicle safety using strict project priority order."""
    return evaluate_safety_policy(
        snapshot=state,
        now_monotonic=now,
        cfg=cfg,
        previous_state=previous_state,
    )


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
        action_executor: SafetyActionExecutor | None = None,
        recorder: SafetyEventRecorder | None = None,
        check_interval_s: float = 0.05,
    ) -> None:
        """Create a safety supervisor."""
        self._state_provider = state_provider
        self._cfg = cfg
        self._motion_gate = motion_gate
        self._logger = logger
        self._cancel_mission = cancel_mission
        self._action_executor = action_executor
        self._recorder = recorder
        self._check_interval_s = check_interval_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._current_state = SafetyState.NORMAL
        self._inhibit_source_state: SafetyState | None = None
        self._inhibit_source_reason: str | None = None
        self._executed_once_keys: set[str] = set()

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
            except Exception as exc:
                self._logger.app_logger.exception("Safety supervisor check failed")
                self._emit(lambda r, e=exc, s=self._current_state: r.record_supervisor_error(
                    state=s.value,
                    reason="supervisor loop exception",
                    error=repr(e),
                ))
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
            self._emit(lambda r, d=decision, p=previous_state: r.record_state_transition(
                state=d.state.value,
                previous_state=p.value if p is not None else None,
                reason=d.reason,
            ))

        if decision.cancel_mission and state_changed:
            self._cancel_mission(decision.reason)
            self._emit(lambda r, d=decision: r.record_mission_cancelled(
                state=d.state.value,
                reason=d.reason,
            ))

        if decision.inhibit_motion:
            self._motion_gate.inhibit(decision.reason)
            self._inhibit_source_state = decision.state
            self._inhibit_source_reason = decision.reason
            self._emit(lambda r, d=decision: r.record_motion_inhibited(
                state=d.state.value,
                reason=d.reason,
            ))
        elif decision.state in {SafetyState.NORMAL, SafetyState.GUIDED_ALLOWED}:
            if self._inhibit_source_state is not None and self._motion_gate.reason() == (self._inhibit_source_reason or ""):
                self._motion_gate.clear()
                self._logger.safety(
                    "motion_gate_cleared",
                    previous_state=self._inhibit_source_state.value,
                )
                self._emit(lambda r, d=decision, s=self._inhibit_source_state: r.record_motion_cleared(
                    state=d.state.value,
                    previous_state=s.value if s is not None else "",
                    reason="safety inhibit cleared after recovery",
                ))
            self._inhibit_source_state = None
            self._inhibit_source_reason = None
            self._executed_once_keys.clear()

        if decision.failsafe_action and state_changed:
            self._logger.safety(
                "failsafe_action_recorded",
                state=decision.state.value,
                action=decision.failsafe_action,
            )

        if self._action_executor is not None:
            self._execute_actions(decision, state_changed=state_changed)

    def _execute_actions(self, decision: SafetyDecision, *, state_changed: bool) -> None:
        for action in decision.actions:
            if action.once_key is not None and action.once_key in self._executed_once_keys and not action.allow_retry:
                self._emit(lambda r, a=action, d=decision: r.record_action_skipped(
                    state=d.state.value,
                    action=a.action,
                    reason=a.reason,
                    once_key=a.once_key or "",
                ))
                continue
            if action.once_key is None and not state_changed and not action.allow_retry:
                continue
            # Tell the executor which safety state we are in so events carry context.
            if self._action_executor is not None:
                self._action_executor._current_state = decision.state.value
            try:
                self._action_executor.execute(action.action, reason=action.reason)
            except Exception as exc:
                self._logger.safety(
                    "safety_action_executor_failed",
                    state=decision.state.value,
                    action=action.action,
                    reason=action.reason,
                    error=repr(exc),
                )
                self._emit(lambda r, e=exc, a=action, d=decision: r.record_supervisor_error(
                    state=d.state.value,
                    reason=f"action executor raised: {a.action}",
                    error=repr(e),
                ))
            finally:
                if action.once_key is not None:
                    self._executed_once_keys.add(action.once_key)


    def _emit(self, fn) -> None:  # type: ignore[type-arg]
        """Call a recorder helper, silencing any exceptions."""
        if self._recorder is None:
            return
        try:
            fn(self._recorder)
        except Exception:  # noqa: BLE001
            pass
