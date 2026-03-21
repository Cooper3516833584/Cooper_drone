from __future__ import annotations

import logging
import threading
import time

from src import control
from src.config_loader import AppConfig
from src.mav_session import SessionLike
from src.mission_base import CancelToken
from src.types import FailsafeAction, SafetyState


class SafetyManager:
    """独立安全线程。优先级: KILL > REVOKE > LINK_LOST > RC_STALE。"""

    def __init__(
        self,
        session: SessionLike,
        cfg: AppConfig,
        cancel_token: CancelToken,
        *,
        logger: logging.Logger | None = None,
    ) -> None:
        self._session = session
        self._cfg = cfg
        self._token = cancel_token
        self._log = logger or logging.getLogger(__name__)

        self._period_s = 1.0 / max(0.1, cfg.safety.poll_hz)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        self._state_lock = threading.RLock()
        self._state = SafetyState.NORMAL
        self._link_lost_action_done = False
        self._last_action_state: SafetyState | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, name="safety-manager", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def state(self) -> SafetyState:
        with self._state_lock:
            return self._state

    def _set_state(self, new_state: SafetyState) -> None:
        with self._state_lock:
            self._state = new_state

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            snapshot = self._session.state_snapshot()
            now = time.monotonic()
            next_state = self._evaluate_state(snapshot, now)
            self._set_state(next_state)
            self._handle_state(next_state)
            time.sleep(self._period_s)

    def _evaluate_state(self, snapshot, now: float) -> SafetyState:
        rc = self._cfg.rc
        channels = snapshot.rc.channels

        kill_main = channels.get(rc.kill_channel)
        kill_aux = channels.get(rc.kill_channel_aux)
        if (kill_main is not None and kill_main >= rc.kill_pwm_threshold) or (
            kill_aux is not None and kill_aux >= rc.kill_pwm_threshold
        ):
            return SafetyState.KILL

        mode_pwm = channels.get(rc.mode_channel)
        if mode_pwm is not None and mode_pwm < rc.guided_pwm_threshold:
            return SafetyState.TAKEOVER_REVOKED

        hb_age = self._session.last_heartbeat_age_s()
        if hb_age > self._cfg.mavlink.heartbeat_timeout_s:
            return SafetyState.LINK_LOST

        rc_age = float("inf")
        if snapshot.rc.last_update_monotonic is not None:
            rc_age = max(0.0, now - snapshot.rc.last_update_monotonic)
        if rc_age > rc.stale_timeout_s:
            return SafetyState.RC_STALE

        if mode_pwm is not None and mode_pwm >= rc.guided_pwm_threshold:
            return SafetyState.GUIDED_ALLOWED
        return SafetyState.NORMAL

    def _handle_state(self, state: SafetyState) -> None:
        if state in {SafetyState.KILL, SafetyState.TAKEOVER_REVOKED, SafetyState.LINK_LOST, SafetyState.RC_STALE}:
            if self._last_action_state == state:
                return
            self._last_action_state = state

        if state == SafetyState.KILL:
            self._handle_kill()
            return
        if state == SafetyState.TAKEOVER_REVOKED:
            self._handle_revoke("rc_takeover_revoked")
            return
        if state == SafetyState.LINK_LOST:
            self._handle_link_lost()
            return
        if state == SafetyState.RC_STALE:
            self._handle_rc_stale()
            return

        # 回到安全状态后允许下次重新触发 link lost 动作。
        self._link_lost_action_done = False
        self._last_action_state = None

    def _handle_kill(self) -> None:
        self._token.cancel("kill_triggered")
        control.inhibit_motion_outputs("kill_triggered")
        control.stop_motion(self._session)
        try:
            control.force_disarm(self._session)
        except Exception as exc:
            self._log.error("force_disarm failed during KILL: %s", exc)
        self._log.critical("Safety KILL triggered")

    def _handle_revoke(self, reason: str) -> None:
        self._token.cancel(reason)
        control.inhibit_motion_outputs(reason)
        control.stop_motion(self._session)
        self._apply_failsafe_mode(self._cfg.failsafe.revoke_action)
        self._log.warning("Safety revoke triggered: %s", reason)

    def _handle_link_lost(self) -> None:
        self._token.cancel("link_lost")
        control.inhibit_motion_outputs("link_lost")
        if self._link_lost_action_done:
            return
        self._link_lost_action_done = True
        self._apply_failsafe_mode(self._cfg.failsafe.link_lost_action)
        self._log.error("Safety link lost triggered")

    def _handle_rc_stale(self) -> None:
        self._token.cancel("rc_stale")
        control.inhibit_motion_outputs("rc_stale")
        control.stop_motion(self._session)
        self._apply_failsafe_mode(self._cfg.failsafe.revoke_action)
        self._log.error("Safety RC stale triggered")

    def _apply_failsafe_mode(self, action: FailsafeAction) -> None:
        try:
            if action == FailsafeAction.LAND:
                control.land_nowait(self._session)
            elif action == FailsafeAction.LOITER:
                control.set_mode_nowait(self._session, "LOITER")
            elif action == FailsafeAction.BRAKE:
                control.set_mode_nowait(self._session, "BRAKE")
            else:
                control.set_mode_nowait(self._session, "LOITER")
        except Exception as exc:
            self._log.error("apply failsafe action failed: %s", exc)
