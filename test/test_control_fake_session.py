"""Tests for legacy control compatibility and fake-session motion gating."""

from __future__ import annotations

from dataclasses import replace
from typing import Sequence

import pytest

from src import control
from src.config_loader import load_config
from src.motion_gate import MotionGate, MotionInhibitedError
from src.movement import DroneMovement
from src.state import VehicleState
from test.conftest import MemoryRuntimeLogger


def test_control_compatibility_functions_delegate_to_default_gate() -> None:
    """Legacy inhibit helpers continue to work through a replaceable MotionGate."""
    gate = MotionGate()
    control.set_default_motion_gate(gate)
    try:
        control.inhibit_motion_outputs("compat")

        assert control.motion_output_inhibited() is True
        assert gate.reason() == "compat"

        control.clear_motion_inhibit()

        assert control.motion_output_inhibited() is False
        gate.assert_motion_allowed()
    finally:
        control.set_default_motion_gate(MotionGate())


def test_send_body_velocity_rejected_when_gate_inhibited() -> None:
    """Normal body velocity output must pass through MotionGate."""
    gate = MotionGate()
    gate.inhibit("blocked")
    movement = _movement(gate)

    with pytest.raises(MotionInhibitedError, match="blocked"):
        movement.send_body_velocity(0.1, 0.0, 0.0)


def test_stop_motion_does_not_propagate_gate_inhibit() -> None:
    """Safety stop can still run while normal motion is inhibited."""
    gate = MotionGate()
    gate.inhibit("blocked")
    commands = FakeCommandService()
    movement = _movement(gate, commands=commands)

    movement.stop_motion()

    assert commands.body_velocity_calls == [("stop_motion", 0.0, 0.0, 0.0, 0.0)]


class FakeCommandService:
    """Fake command service that captures velocity setpoints."""

    def __init__(self) -> None:
        self.body_velocity_calls: list[tuple] = []

    def command_long(
        self,
        command: int,
        params: Sequence[float] = (),
        *,
        timeout_s: float | None = None,
        retries: int | None = None,
        name: str | None = None,
    ) -> None:
        pass

    def send_body_velocity_setpoint(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
        *,
        name: str = "body_velocity_setpoint",
    ) -> None:
        self.body_velocity_calls.append((name, vx_mps, vy_mps, vz_mps, yaw_rate_dps))


def _movement(gate: MotionGate, *, commands: FakeCommandService | None = None) -> DroneMovement:
    cfg = load_config("config/dry_run.yaml")
    cfg = replace(
        cfg,
        dry_run=False,
        mavlink=replace(cfg.mavlink, connection_string="udp:127.0.0.1:14550"),
    )
    return DroneMovement(
        cfg,
        commands or FakeCommandService(),
        lambda: VehicleState(mode="GUIDED", armed=True),
        gate,
        MemoryRuntimeLogger(),
    )
