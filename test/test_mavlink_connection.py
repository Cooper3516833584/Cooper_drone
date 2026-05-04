"""Tests for the low-level MAVLink connection wrapper."""

from __future__ import annotations

import logging
import math
from pathlib import Path
from uuid import uuid4

import pytest

from src.config_loader import MavlinkConfig
from src.logging_runtime import RuntimeLogger
from src.mavlink_connection import MavlinkConnection
from test.fake_mavlink import FakeMavlinkConnection, FakeMavlinkMessage


def test_dry_run_does_not_call_real_pymavlink() -> None:
    """Skip connection creation for dry-run configuration."""
    called = False

    def factory(*args, **kwargs):
        nonlocal called
        called = True
        return FakeMavlinkConnection()

    connection = MavlinkConnection(_mavlink_config("dry-run"), _fake_runtime_logger(), factory)
    connection.connect()

    assert called is False
    assert connection.state_snapshot().mode is None


def test_receiver_thread_can_start_and_stop() -> None:
    """Start and stop the receiver thread using a fake connection."""
    fake_connection = FakeMavlinkConnection(
        [
            FakeMavlinkMessage(
                "GLOBAL_POSITION_INT",
                relative_alt=2500,
                lat=31_0000000,
                lon=121_0000000,
                vx=10,
                vy=20,
                vz=-30,
            )
        ]
    )

    connection = MavlinkConnection(
        _mavlink_config("udp:127.0.0.1:14550"),
        _fake_runtime_logger(),
        lambda *args, **kwargs: fake_connection,
    )
    connection.connect()
    connection.start_receiver()
    connection.stop_receiver()
    state = connection.state_snapshot()
    connection.close()

    assert state.relative_alt_m == 2.5
    assert fake_connection.closed is True


@pytest.mark.parametrize(
    ("yaw_rate_dps", "expected_rad_s"),
    [
        (90.0, math.pi / 2),
        (-45.0, -math.pi / 4),
    ],
)
def test_body_velocity_converts_yaw_rate_dps_to_rad_s(
    yaw_rate_dps: float,
    expected_rad_s: float,
) -> None:
    """Convert upper API yaw rate units before sending MAVLink setpoints."""
    fake_connection = FakeMavlinkConnection()
    connection = MavlinkConnection(
        _mavlink_config("udp:127.0.0.1:14550"),
        _fake_runtime_logger(),
        lambda *args, **kwargs: fake_connection,
    )
    try:
        connection.connect()
        connection.send_body_velocity_setpoint(0.0, 0.0, 0.0, yaw_rate_dps)
    finally:
        connection.close()

    sent_yaw_rate = fake_connection.mav.body_velocity_calls[-1]["yaw_rate_rad_s"]
    assert sent_yaw_rate == pytest.approx(expected_rad_s, rel=1e-6)


def _mavlink_config(connection_string: str) -> MavlinkConfig:
    return MavlinkConfig(
        connection_string=connection_string,
        baud=115200,
        source_system=255,
        source_component=0,
        target_system=None,
        target_component=None,
        heartbeat_timeout_s=1.0,
        command_timeout_s=1.0,
        command_retries=1,
    )


def _fake_runtime_logger() -> RuntimeLogger:
    logger = logging.getLogger(f"test.runtime.{uuid4().hex}")
    logger.addHandler(logging.NullHandler())
    return RuntimeLogger(
        run_id="test-run",
        run_dir=Path("logs/test-runtime"),
        app_logger=logger,
    )
