from __future__ import annotations

import time
from pathlib import Path

from src import control
from src.config_loader import load_config
from src.mav_session import FakeMavSession
from src.mav_state import VehicleStateCache
from src.mission_base import CancelToken
from src.safety import SafetyManager
from src.types import SafetyState

DATA = Path(__file__).parent / "data"


def _build():
    cfg = load_config(DATA / "cfg_dry_run.yaml")
    cache = VehicleStateCache()
    session = FakeMavSession(cfg.mavlink, cache)
    session.connect()
    control.bind_runtime_limits(cfg.limits)
    control.clear_motion_inhibit()
    token = CancelToken()
    mgr = SafetyManager(session, cfg, token)
    return cfg, cache, session, token, mgr


def _wait_until(predicate, timeout_s: float = 2.0) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_kill_trigger() -> None:
    cfg, cache, _session, token, mgr = _build()
    mgr.start()
    cache.update_rc_channels({cfg.rc.mode_channel: 1800, cfg.rc.kill_channel: 1900, cfg.rc.kill_channel_aux: 1000})
    assert _wait_until(lambda: token.is_cancelled())
    assert mgr.state() == SafetyState.KILL
    assert control.motion_output_inhibited() is True
    mgr.stop()
    control.clear_motion_inhibit()


def test_revoke_trigger() -> None:
    cfg, cache, _session, token, mgr = _build()
    mgr.start()
    cache.update_rc_channels({cfg.rc.mode_channel: 1200, cfg.rc.kill_channel: 1000, cfg.rc.kill_channel_aux: 1000})
    assert _wait_until(lambda: token.is_cancelled())
    assert mgr.state() == SafetyState.TAKEOVER_REVOKED
    mgr.stop()
    control.clear_motion_inhibit()


def test_link_lost_trigger() -> None:
    _cfg, cache, session, token, mgr = _build()
    session.auto_heartbeat = False

    def _make_old_hb(state):
        state.last_heartbeat_monotonic = time.monotonic() - 10.0

    cache.update(_make_old_hb)
    mgr.start()
    assert _wait_until(lambda: token.is_cancelled())
    assert mgr.state() == SafetyState.LINK_LOST
    mgr.stop()
    control.clear_motion_inhibit()


def test_rc_stale_trigger_and_motion_inhibit_effective() -> None:
    cfg, cache, session, token, mgr = _build()
    session.auto_heartbeat = True

    def _stale_rc(state):
        state.rc.last_update_monotonic = time.monotonic() - (cfg.rc.stale_timeout_s + 3.0)

    cache.update(_stale_rc)
    mgr.start()
    assert _wait_until(lambda: token.is_cancelled())
    assert mgr.state() == SafetyState.RC_STALE

    try:
        control.send_body_velocity(session, 0.2, 0.0, 0.0)
        assert False, "motion output should be inhibited after RC stale"
    except RuntimeError:
        pass

    mgr.stop()
    control.clear_motion_inhibit()

