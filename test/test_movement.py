"""Tests for low-level movement control."""

from __future__ import annotations

import logging
from dataclasses import replace
from pathlib import Path
from typing import Sequence

import pytest

from src.config_loader import AppConfig, MavlinkConfig, SafetyConfig, load_config
from src.logging_runtime import RuntimeLogger
from src.motion_gate import MotionGate, MotionInhibitedError
from src.movement import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    DroneMovement,
    MovementRejected,
)
from src.state import VehicleState


def test_dry_run_arm_logs_without_command_send() -> None:
    """Dry-run arm logs success without calling command service."""
    logger = _memory_logger()
    commands = FakeCommandService()
    movement = _movement(_cfg(dry_run=True, allow_arm=True), commands, VehicleState(mode="GUIDED"), logger=logger)

    movement.arm()

    assert commands.command_long_calls == []
    assert any(record["name"] == "arm" and record["fields"]["status"] == "dry_run" for record in logger.records)


def test_arm_rejects_when_allow_arm_false() -> None:
    """Reject arming when safety config disables arming."""
    movement = _movement(_cfg(dry_run=True, allow_arm=False), FakeCommandService(), VehicleState(mode="GUIDED"))

    with pytest.raises(MovementRejected, match="Arming is disabled"):
        movement.arm()


def test_takeoff_rejects_altitude_over_limit() -> None:
    """Reject takeoff above configured maximum altitude."""
    movement = _movement(_cfg(allow_arm=True), FakeCommandService(), VehicleState(mode="GUIDED", armed=True))

    with pytest.raises(MovementRejected, match="max_altitude"):
        movement.takeoff(999.0)


def test_takeoff_arms_when_not_armed() -> None:
    """Call arm before takeoff when vehicle is not armed."""
    state = VehicleState(mode="GUIDED", armed=False, relative_alt_m=0.0)
    commands = FakeCommandService(state)
    movement = _movement(_cfg(allow_arm=True), commands, state)

    movement.takeoff(2.0)

    sent_commands = [call["command"] for call in commands.command_long_calls]
    assert sent_commands == [MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_NAV_TAKEOFF]
    assert state.armed is True


def test_takeoff_waits_until_altitude_reached() -> None:
    """Wait until relative altitude reaches the takeoff threshold."""
    state = VehicleState(mode="GUIDED", armed=True, relative_alt_m=0.0)
    commands = FakeCommandService(state)
    movement = _movement(_cfg(allow_arm=True), commands, state)

    movement.takeoff(3.0)

    assert state.relative_alt_m == 3.0


def test_send_body_velocity_clamps_values() -> None:
    """Clamp velocity and yaw-rate setpoints before sending."""
    commands = FakeCommandService()
    movement = _movement(_cfg(), commands, VehicleState(mode="GUIDED"))

    movement.send_body_velocity(99.0, -99.0, 99.0, 999.0)

    call = commands.body_velocity_calls[-1]
    assert call == {
        "vx_mps": 2.0,
        "vy_mps": -2.0,
        "vz_mps": 1.0,
        "yaw_rate_dps": 45.0,
        "name": "body_velocity_setpoint",
    }


def test_motion_inhibited_blocks_body_velocity() -> None:
    """Block body velocity when the motion gate is inhibited."""
    gate = MotionGate()
    gate.inhibit("blocked")
    movement = _movement(_cfg(), FakeCommandService(), VehicleState(mode="GUIDED"), gate=gate)

    with pytest.raises(MotionInhibitedError, match="blocked"):
        movement.send_body_velocity(0.1, 0.0, 0.0)


def test_land_brake_loiter_allowed_when_inhibited() -> None:
    """Allow safety actions even when the motion gate is inhibited."""
    gate = MotionGate()
    gate.inhibit("blocked")
    commands = FakeCommandService()
    movement = _movement(_cfg(), commands, VehicleState(mode="GUIDED"), gate=gate)

    movement.land()
    movement.brake()
    movement.loiter()

    sent_commands = [call["command"] for call in commands.command_long_calls]
    assert sent_commands == [MAV_CMD_NAV_LAND, MAV_CMD_DO_SET_MODE, MAV_CMD_DO_SET_MODE]


def test_disarm_allowed_when_inhibited() -> None:
    """Allow disarm even when the motion gate is inhibited."""
    gate = MotionGate()
    gate.inhibit("blocked")
    state = VehicleState(mode="GUIDED", armed=True)
    commands = FakeCommandService(state)
    movement = _movement(_cfg(), commands, state, gate=gate)

    movement.disarm()

    assert state.armed is False
    assert commands.command_long_calls[-1]["params"][0] == 0.0


def test_stop_motion_does_not_propagate_inhibited_error() -> None:
    """Do not propagate gate inhibition from stop_motion."""
    gate = MotionGate()
    gate.inhibit("blocked")
    commands = FakeCommandService()
    movement = _movement(_cfg(), commands, VehicleState(mode="GUIDED"), gate=gate)

    movement.stop_motion()

    assert commands.body_velocity_calls[-1]["name"] == "stop_motion"


class FakeCommandService:
    """Fake command service for movement tests."""

    def __init__(self, state: VehicleState | None = None) -> None:
        """Create a fake command service."""
        self.state = state
        self.command_long_calls: list[dict] = []
        self.body_velocity_calls: list[dict] = []

    def command_long(
        self,
        command: int,
        params: Sequence[float] = (),
        *,
        timeout_s: float | None = None,
        retries: int | None = None,
        name: str | None = None,
    ):
        """Capture a COMMAND_LONG request and update fake state."""
        params_list = [float(value) for value in params]
        self.command_long_calls.append({"command": command, "params": params_list, "name": name})
        if self.state is not None:
            if command == MAV_CMD_COMPONENT_ARM_DISARM:
                self.state.armed = params_list[0] == 1.0
            elif command == MAV_CMD_NAV_TAKEOFF:
                self.state.relative_alt_m = params_list[6]

    def send_body_velocity_setpoint(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
        *,
        name: str = "body_velocity_setpoint",
    ) -> None:
        """Capture a body velocity setpoint."""
        self.body_velocity_calls.append(
            {
                "vx_mps": vx_mps,
                "vy_mps": vy_mps,
                "vz_mps": vz_mps,
                "yaw_rate_dps": yaw_rate_dps,
                "name": name,
            }
        )


class MemoryRuntimeLogger(RuntimeLogger):
    """Runtime logger that stores events in memory."""

    def __init__(self) -> None:
        """Create an in-memory runtime logger."""
        super().__init__(
            run_id="test-run",
            run_dir=Path("logs/test-movement"),
            app_logger=logging.getLogger("test.movement"),
        )
        self.records: list[dict] = []

    def event(self, name: str, **fields) -> None:
        """Capture an event record."""
        self.records.append({"type": "event", "name": name, "fields": fields})

    def command(self, name: str, **fields) -> None:
        """Capture a command record."""
        self.records.append({"type": "command", "name": name, "fields": fields})


def _movement(
    cfg: AppConfig,
    commands: FakeCommandService,
    state: VehicleState,
    *,
    gate: MotionGate | None = None,
    logger: MemoryRuntimeLogger | None = None,
) -> DroneMovement:
    return DroneMovement(
        cfg,
        commands,
        lambda: state,
        gate or MotionGate(),
        logger or _memory_logger(),
    )


def _cfg(*, dry_run: bool = False, allow_arm: bool = False) -> AppConfig:
    cfg = load_config("config/dry_run.yaml")
    return replace(
        cfg,
        dry_run=dry_run,
        mavlink=replace(
            cfg.mavlink,
            connection_string="dry-run" if dry_run else "udp:127.0.0.1:14550",
            command_timeout_s=0.2,
        ),
        safety=replace(cfg.safety, allow_arm=allow_arm, require_guided=True),
    )


def _memory_logger() -> MemoryRuntimeLogger:
    return MemoryRuntimeLogger()
