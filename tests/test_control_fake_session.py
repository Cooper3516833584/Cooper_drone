from __future__ import annotations

from pathlib import Path

from src import control
from src.config_loader import load_config
from src.mav_session import FakeMavSession
from src.mav_state import VehicleStateCache

DATA = Path(__file__).parent / "data"


def _session():
    cfg = load_config(DATA / "cfg_dry_run.yaml")
    cache = VehicleStateCache()
    session = FakeMavSession(cfg.mavlink, cache)
    session.connect()
    control.bind_runtime_limits(cfg.limits)
    control.clear_motion_inhibit()
    return cfg, session


def test_primitives_and_motion_limit() -> None:
    cfg, session = _session()

    control.set_mode(session, "GUIDED")
    control.arm(session)
    control.takeoff(session, alt_m=cfg.limits.takeoff_alt_m, timeout_s=5.0)
    control.goto_global(session, 30.2000000, 120.2000000, 2.0, timeout_s=1.0)
    control.send_body_velocity(session, 5.0, -5.0, 3.0, yaw_rate=300.0)
    control.stop_motion(session)
    control.land(session, timeout_s=5.0)
    control.disarm(session)

    last_vel = session.velocity_history[-4]  # stop_motion 前一条速度
    assert abs(last_vel[0]) <= cfg.limits.max_xy_vel_mps
    assert abs(last_vel[1]) <= cfg.limits.max_xy_vel_mps
    assert abs(last_vel[2]) <= cfg.limits.max_z_vel_mps
    assert abs(last_vel[3]) <= 3.1415927  # yaw_rate 限幅后转弧度

    state = session.state_snapshot()
    assert state.mode == "LAND"
    assert state.armed is False
    assert len(session.velocity_history) >= 4
    assert session.goto_history[-1][0] == 30.2


def test_motion_inhibit_blocks_new_outputs() -> None:
    _cfg, session = _session()
    control.inhibit_motion_outputs("test")
    try:
        control.send_body_velocity(session, 0.5, 0.0, 0.0)
        assert False, "send_body_velocity should fail when inhibited"
    except RuntimeError:
        pass
    finally:
        control.clear_motion_inhibit()

