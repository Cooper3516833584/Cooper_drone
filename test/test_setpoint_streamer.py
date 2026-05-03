"""Tests for body velocity setpoint streamer."""

from __future__ import annotations

import logging
import time
from pathlib import Path

import pytest

from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionInhibitedError
from src.setpoint_streamer import BodyVelocityStreamer


def test_start_runs_thread() -> None:
    """Start the streamer thread."""
    movement = FakeMovement()
    streamer = BodyVelocityStreamer(movement, MemoryRuntimeLogger(), rate_hz=20.0, setpoint_ttl_s=0.2)

    streamer.start()
    try:
        assert streamer.is_running()
    finally:
        streamer.stop()


def test_update_periodically_calls_send_body_velocity() -> None:
    """Send the latest setpoint periodically after update."""
    movement = FakeMovement()
    streamer = BodyVelocityStreamer(movement, MemoryRuntimeLogger(), rate_hz=40.0, setpoint_ttl_s=0.5)

    streamer.start()
    streamer.update(1.0, 2.0, -0.5, 10.0)
    _wait_until(lambda: len([call for call in movement.calls if call == (1.0, 2.0, -0.5, 10.0)]) >= 2)
    streamer.stop()

    assert (1.0, 2.0, -0.5, 10.0) in movement.calls


def test_ttl_timeout_sends_zero_velocity() -> None:
    """Send zero velocity after the setpoint TTL expires."""
    movement = FakeMovement()
    streamer = BodyVelocityStreamer(movement, MemoryRuntimeLogger(), rate_hz=30.0, setpoint_ttl_s=0.05)

    streamer.start()
    streamer.update(1.0, 0.0, 0.0, 0.0)
    _wait_until(lambda: (0.0, 0.0, 0.0, 0.0) in movement.calls)
    streamer.stop()

    assert (0.0, 0.0, 0.0, 0.0) in movement.calls


def test_stop_sends_zero_and_exits() -> None:
    """Send one zero velocity during stop and exit the thread."""
    movement = FakeMovement()
    streamer = BodyVelocityStreamer(movement, MemoryRuntimeLogger(), rate_hz=20.0, setpoint_ttl_s=0.2)

    streamer.start()
    streamer.update(1.0, 0.0, 0.0, 0.0)
    streamer.stop()

    assert not streamer.is_running()
    assert (0.0, 0.0, 0.0, 0.0) in movement.calls


def test_motion_inhibited_does_not_kill_thread() -> None:
    """Keep the streamer alive after motion inhibition errors."""
    movement = FakeMovement(fail_count=3)
    logger = MemoryRuntimeLogger()
    streamer = BodyVelocityStreamer(movement, logger, rate_hz=30.0, setpoint_ttl_s=0.2)

    streamer.start()
    streamer.update(1.0, 0.0, 0.0, 0.0)
    _wait_until(lambda: any(record["name"] == "body_velocity_streamer_send_failed" for record in logger.records))
    assert streamer.is_running()
    streamer.stop()


def test_multiple_stop_calls_are_safe() -> None:
    """Allow repeated stop calls."""
    movement = FakeMovement()
    streamer = BodyVelocityStreamer(movement, MemoryRuntimeLogger(), rate_hz=20.0, setpoint_ttl_s=0.2)

    streamer.start()
    streamer.stop()
    streamer.stop()

    assert not streamer.is_running()


def test_invalid_rate_hz_raises() -> None:
    """Reject nonpositive stream rates."""
    with pytest.raises(ValueError, match="rate_hz"):
        BodyVelocityStreamer(FakeMovement(), MemoryRuntimeLogger(), rate_hz=0.0, setpoint_ttl_s=0.2)


def test_invalid_setpoint_ttl_raises() -> None:
    """Reject nonpositive setpoint TTL values."""
    with pytest.raises(ValueError, match="setpoint_ttl_s"):
        BodyVelocityStreamer(FakeMovement(), MemoryRuntimeLogger(), rate_hz=10.0, setpoint_ttl_s=0.0)


class FakeMovement:
    """Fake movement API for streamer tests."""

    def __init__(self, *, fail_count: int = 0) -> None:
        """Create fake movement."""
        self.calls: list[tuple[float, float, float, float]] = []
        self.fail_count = fail_count

    def send_body_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float = 0.0,
    ) -> None:
        """Capture a velocity setpoint or raise a motion gate error."""
        self.calls.append((vx_mps, vy_mps, vz_mps, yaw_rate_dps))
        if self.fail_count > 0:
            self.fail_count -= 1
            raise MotionInhibitedError("motion inhibited")


class MemoryRuntimeLogger(RuntimeLogger):
    """Runtime logger that stores events in memory."""

    def __init__(self) -> None:
        """Create an in-memory runtime logger."""
        super().__init__(
            run_id="test-run",
            run_dir=Path("logs/test-setpoint-streamer"),
            app_logger=logging.getLogger("test.setpoint_streamer"),
        )
        self.records: list[dict] = []

    def event(self, name: str, **fields) -> None:
        """Capture an event record."""
        self.records.append({"name": name, "fields": fields})


def _wait_until(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for predicate")
