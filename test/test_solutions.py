"""Tests for high-level mission solutions."""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from src.config_loader import load_config
from src.logging_runtime import RuntimeLogger
from src.mission_runtime import CancellationToken, MissionCancelled
from src.solutions.common import SolutionContext
from src.solutions.takeoff_and_hover import run_takeoff_and_hover
from src.solutions.vision_follow import run_vision_follow
from src.solutions.waypoint_square import run_waypoint_square
from src.state import VehicleState


def test_takeoff_and_hover_calls_expected_movement_methods() -> None:
    """Call movement set_mode, arm, takeoff, and land."""
    movement = FakeMovement()
    ctx = _context(movement)

    run_takeoff_and_hover(ctx, altitude_m=2.0, hover_s=0.0)

    assert movement.calls == [
        ("set_mode", "GUIDED"),
        ("arm",),
        ("takeoff", 2.0),
        ("land",),
    ]


def test_takeoff_and_hover_exits_when_token_cancelled() -> None:
    """Propagate mission cancellation from a cancelled token."""
    movement = FakeMovement()
    token = CancellationToken()
    token.cancel("cancelled by test")
    ctx = _context(movement, token=token)

    with pytest.raises(MissionCancelled, match="cancelled by test"):
        run_takeoff_and_hover(ctx, altitude_m=2.0, hover_s=0.0)

    assert ("land",) in movement.calls


def test_waypoint_square_sends_four_velocity_segments() -> None:
    """Send four timed body velocity segments for a square."""
    movement = FakeMovement()
    ctx = _context(movement)

    run_waypoint_square(ctx, altitude_m=2.0, side_m=1.0, speed_mps=1000.0)

    velocity_calls = [call for call in movement.calls if call[0] == "send_body_velocity"]
    assert velocity_calls == [
        ("send_body_velocity", 1000.0, 0.0, 0.0, 0.0),
        ("send_body_velocity", 0.0, 1000.0, 0.0, 0.0),
        ("send_body_velocity", -1000.0, 0.0, 0.0, 0.0),
        ("send_body_velocity", 0.0, -1000.0, 0.0, 0.0),
    ]


def test_waypoint_square_stops_after_each_segment() -> None:
    """Call stop_motion after each square segment."""
    movement = FakeMovement()
    ctx = _context(movement)

    run_waypoint_square(ctx, altitude_m=2.0, side_m=1.0, speed_mps=1000.0)

    assert [call for call in movement.calls if call == ("stop_motion",)] == [
        ("stop_motion",),
        ("stop_motion",),
        ("stop_motion",),
        ("stop_motion",),
    ]


def test_vision_follow_starts_and_stops_streamer(monkeypatch) -> None:
    """Start and stop the body velocity streamer in vision follow."""
    created_streamers: list[FakeStreamer] = []

    def fake_streamer_factory(*args, **kwargs):
        streamer = FakeStreamer()
        created_streamers.append(streamer)
        return streamer

    monkeypatch.setattr("src.solutions.vision_follow.BodyVelocityStreamer", fake_streamer_factory)
    movement = FakeMovement()
    ctx = _context(movement)

    run_vision_follow(ctx, altitude_m=2.0, duration_s=0.0)

    assert len(created_streamers) == 1
    assert created_streamers[0].calls == [
        ("start",),
        ("update", 0.0, 0.0, 0.0, 0.0),
        ("stop",),
    ]


def test_solutions_do_not_contain_forbidden_mavlink_strings() -> None:
    """Keep high-level solutions free of low-level transport calls."""
    forbidden = [
        "pymavlink",
        "mavutil",
        "command_long_send",
        "set_position_target",
        "recv_match",
    ]
    for path in Path("src/solutions").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        for term in forbidden:
            assert term not in source


class FakeMovement:
    """Fake movement API for solution tests."""

    def __init__(self) -> None:
        """Create fake movement."""
        self.calls: list[tuple] = []

    def set_mode(self, mode: str) -> None:
        """Capture set mode."""
        self.calls.append(("set_mode", mode))

    def arm(self) -> None:
        """Capture arm."""
        self.calls.append(("arm",))

    def takeoff(self, altitude_m: float) -> None:
        """Capture takeoff."""
        self.calls.append(("takeoff", altitude_m))

    def land(self) -> None:
        """Capture land."""
        self.calls.append(("land",))

    def stop_motion(self) -> None:
        """Capture stop motion."""
        self.calls.append(("stop_motion",))

    def send_body_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float = 0.0,
    ) -> None:
        """Capture body velocity."""
        self.calls.append(("send_body_velocity", vx_mps, vy_mps, vz_mps, yaw_rate_dps))


class FakeStreamer:
    """Fake body velocity streamer."""

    def __init__(self) -> None:
        """Create fake streamer."""
        self.calls: list[tuple] = []

    def start(self) -> None:
        """Capture start."""
        self.calls.append(("start",))

    def update(self, vx_mps: float, vy_mps: float, vz_mps: float, yaw_rate_dps: float = 0.0) -> None:
        """Capture update."""
        self.calls.append(("update", vx_mps, vy_mps, vz_mps, yaw_rate_dps))

    def stop(self) -> None:
        """Capture stop."""
        self.calls.append(("stop",))


class MemoryRuntimeLogger(RuntimeLogger):
    """Runtime logger that stores solution events in memory."""

    def __init__(self) -> None:
        """Create an in-memory runtime logger."""
        super().__init__(
            run_id="test-run",
            run_dir=Path("logs/test-solutions"),
            app_logger=logging.getLogger("test.solutions"),
        )
        self.events: list[tuple[str, dict]] = []

    def event(self, name: str, **fields) -> None:
        """Capture an event."""
        self.events.append((name, fields))


def _context(movement: FakeMovement, *, token: CancellationToken | None = None) -> SolutionContext:
    state = VehicleState(mode="GUIDED", armed=True, relative_alt_m=2.0)
    return SolutionContext(
        cfg=load_config("config/dry_run.yaml"),
        movement=movement,
        logger=MemoryRuntimeLogger(),
        token=token or CancellationToken(),
        state_provider=lambda: state,
    )
