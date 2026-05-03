"""Tests for vehicle state parsing."""

from __future__ import annotations

from src.state import VehicleStateCache
from test.fake_mavlink import FakeMavlinkMessage


def test_vehicle_state_cache_parses_heartbeat() -> None:
    """Parse HEARTBEAT into mode, armed state, and heartbeat time."""
    cache = VehicleStateCache()
    msg = FakeMavlinkMessage(
        "HEARTBEAT",
        base_mode=128,
        custom_mode=4,
        flightmode="GUIDED",
        system_status="4",
    )

    cache.update_from_message(msg, now=123.0)
    state = cache.snapshot()

    assert state.last_heartbeat_ts == 123.0
    assert state.mode == "GUIDED"
    assert state.armed is True
    assert state.system_status == "4"


def test_vehicle_state_cache_parses_global_position_int() -> None:
    """Parse GLOBAL_POSITION_INT into metric and degree fields."""
    cache = VehicleStateCache()
    msg = FakeMavlinkMessage(
        "GLOBAL_POSITION_INT",
        relative_alt=1234,
        lat=31_1234567,
        lon=121_7654321,
        vx=150,
        vy=-50,
        vz=25,
    )

    cache.update_from_message(msg)
    state = cache.snapshot()

    assert state.relative_alt_m == 1.234
    assert state.lat_deg == 31.1234567
    assert state.lon_deg == 121.7654321
    assert state.vx_mps == 1.5
    assert state.vy_mps == -0.5
    assert state.vz_mps == 0.25


def test_snapshot_does_not_expose_internal_rc_channels() -> None:
    """Keep the internal RC channel dictionary private."""
    cache = VehicleStateCache()
    msg = FakeMavlinkMessage("RC_CHANNELS", chan1_raw=1000, chan2_raw=1500)
    cache.update_from_message(msg, now=10.0)

    snapshot = cache.snapshot()
    snapshot.rc_channels[1] = 2000

    assert cache.snapshot().rc_channels[1] == 1000
