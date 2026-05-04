"""Tests for MAVLink message parsing helpers."""

from __future__ import annotations

import pytest

from src.mavlink_messages import MAV_MODE_FLAG_SAFETY_ARMED, heartbeat_to_mode_and_armed
from test.fake_mavlink import FakeMavlinkMessage


@pytest.mark.parametrize(
    ("custom_mode", "expected_mode"),
    [
        (4, "GUIDED"),
        (9, "LAND"),
        (17, "BRAKE"),
    ],
)
def test_copter_custom_mode_maps_to_mode_name(custom_mode: int, expected_mode: str) -> None:
    msg = FakeMavlinkMessage("HEARTBEAT", base_mode=0, custom_mode=custom_mode)

    mode, _armed = heartbeat_to_mode_and_armed(msg)

    assert mode == expected_mode


def test_unknown_custom_mode_falls_back_to_raw_value() -> None:
    msg = FakeMavlinkMessage("HEARTBEAT", base_mode=0, custom_mode=999)

    mode, _armed = heartbeat_to_mode_and_armed(msg)

    assert mode == "custom_mode:999"


def test_custom_mode_mapping_preserves_armed_state() -> None:
    msg = FakeMavlinkMessage(
        "HEARTBEAT",
        base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
        custom_mode=4,
    )

    mode, armed = heartbeat_to_mode_and_armed(msg)

    assert mode == "GUIDED"
    assert armed is True
