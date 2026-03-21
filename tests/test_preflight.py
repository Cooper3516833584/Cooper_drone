from __future__ import annotations

import time
from pathlib import Path

import pytest

from src import control
from src.config_loader import load_config
from src.mav_session import FakeMavSession
from src.mav_state import VehicleStateCache

DATA = Path(__file__).parent / "data"


def _ready_session():
    cfg = load_config(DATA / "cfg_dry_run.yaml")
    cache = VehicleStateCache()
    session = FakeMavSession(cfg.mavlink, cache)
    session.connect()
    control.bind_runtime_limits(cfg.limits)
    control.clear_motion_inhibit()
    return cfg, cache, session


def test_preflight_ok() -> None:
    cfg, _cache, session = _ready_session()
    control.preflight_check(session, cfg)


def test_preflight_fail_gps_missing() -> None:
    cfg, cache, session = _ready_session()
    cache.update_gps(fix_type=None, satellites_visible=None)
    with pytest.raises(RuntimeError):
        control.preflight_check(session, cfg)


def test_preflight_fail_rc_stale() -> None:
    cfg, cache, session = _ready_session()

    def _stale(state):
        state.rc.last_update_monotonic = time.monotonic() - (cfg.rc.stale_timeout_s + 2.0)

    cache.update(_stale)
    with pytest.raises(RuntimeError):
        control.preflight_check(session, cfg)


def test_preflight_fail_battery() -> None:
    cfg, cache, session = _ready_session()
    cache.update_battery(voltage_v=10.0, current_a=0.0, remaining_pct=5)
    with pytest.raises(RuntimeError):
        control.preflight_check(session, cfg)


def test_preflight_fail_ekf() -> None:
    cfg, cache, session = _ready_session()
    cache.update_ekf_health(ekf_healthy=False)
    with pytest.raises(RuntimeError):
        control.preflight_check(session, cfg)

