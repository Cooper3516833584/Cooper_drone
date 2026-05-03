"""Low-level movement control API."""

from __future__ import annotations

import time
from typing import Callable

from src.config_loader import AppConfig
from src.logging_runtime import RuntimeLogger
from src.mavlink_commands import (
    MavlinkCommandError,
    MavlinkCommandRejected,
    MavlinkCommandService,
    MavlinkCommandTimeout,
)
from src.motion_gate import MotionGate
from src.state import VehicleState


MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

COPTER_MODES = {
    "STABILIZE": 0,
    "ACRO": 1,
    "ALT_HOLD": 2,
    "AUTO": 3,
    "GUIDED": 4,
    "LOITER": 5,
    "RTL": 6,
    "CIRCLE": 7,
    "LAND": 9,
    "BRAKE": 17,
    "GUIDED_NOGPS": 20,
}


class MovementError(RuntimeError):
    """Raised when a movement operation fails."""


class MovementTimeout(MovementError):
    """Raised when vehicle state does not reach the expected condition."""


class MovementRejected(MovementError):
    """Raised when a movement command is rejected."""


class DroneMovement:
    """Low-level movement API with limits, safety gate, and logging."""

    def __init__(
        self,
        cfg: AppConfig,
        commands: MavlinkCommandService,
        state_provider: Callable[[], VehicleState],
        motion_gate: MotionGate,
        logger: RuntimeLogger,
    ) -> None:
        """Create a low-level movement controller."""
        self._cfg = cfg
        self._commands = commands
        self._state_provider = state_provider
        self._motion_gate = motion_gate
        self._logger = logger

    def set_mode(self, mode: str) -> None:
        """Set the flight mode and wait until state reports the target mode."""
        target_mode = _normalize_mode(mode)
        self._logger.event("set_mode_started", mode=target_mode)

        if self._cfg.dry_run:
            self._logger.command("set_mode", mode=target_mode, status="dry_run")
            self._logger.event("set_mode_finished", mode=target_mode, status="dry_run")
            return

        mode_number = _mode_number(target_mode)
        self._command_long(
            MAV_CMD_DO_SET_MODE,
            [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_number],
            name=f"set_mode_{target_mode.lower()}",
        )
        self._wait_for(lambda state: _normalize_mode(state.mode) == target_mode, f"mode {target_mode}")
        self._logger.event("set_mode_finished", mode=target_mode, status="ok")

    def arm(self) -> None:
        """Arm the vehicle if allowed by configuration."""
        if not self._cfg.safety.allow_arm:
            raise MovementRejected("Arming is disabled by safety configuration")

        self._logger.event("arm_started")
        if self._cfg.dry_run:
            self._logger.command("arm", status="dry_run")
            self._logger.event("arm_finished", status="dry_run")
            return

        self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, [1.0], name="arm")
        self._wait_for(lambda state: state.armed is True, "armed state true")
        self._logger.event("arm_finished", status="ok")

    def disarm(self) -> None:
        """Disarm the vehicle even if movement is inhibited."""
        self._logger.event("disarm_started")
        if self._cfg.dry_run:
            self._logger.command("disarm", status="dry_run")
            self._logger.event("disarm_finished", status="dry_run")
            return

        self._command_long(MAV_CMD_COMPONENT_ARM_DISARM, [0.0], name="disarm")
        self._wait_for(lambda state: state.armed is False, "armed state false")
        self._logger.event("disarm_finished", status="ok")

    def takeoff(self, altitude_m: float) -> None:
        """Command takeoff to a target relative altitude."""
        self._motion_gate.assert_motion_allowed()
        if altitude_m <= 0:
            raise MovementRejected("Takeoff altitude must be greater than 0")
        if altitude_m > self._cfg.limits.max_altitude_m:
            raise MovementRejected("Takeoff altitude exceeds configured max_altitude_m")
        if self._cfg.safety.require_guided and _normalize_mode(self._state_provider().mode) != "GUIDED":
            raise MovementRejected("Takeoff requires GUIDED mode")

        self._logger.event("takeoff_started", altitude_m=altitude_m)
        if self._cfg.dry_run:
            if self._state_provider().armed is not True:
                self.arm()
            self._logger.command("takeoff", altitude_m=altitude_m, status="dry_run")
            self._logger.event("takeoff_finished", altitude_m=altitude_m, status="dry_run")
            return

        if self._state_provider().armed is not True:
            self.arm()

        self._command_long(MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_m], name="takeoff")
        threshold = altitude_m - self._cfg.limits.takeoff_altitude_tolerance_m
        self._wait_for(lambda state: (state.relative_alt_m or 0.0) >= threshold, f"altitude {threshold}")
        self._logger.event("takeoff_finished", altitude_m=altitude_m, status="ok")

    def land(self) -> None:
        """Command land as a safety action."""
        self._logger.event("land_started")
        if self._cfg.dry_run:
            self._logger.command("land", status="dry_run")
            self._logger.event("land_finished", status="dry_run")
            return
        self._command_long(MAV_CMD_NAV_LAND, name="land")
        self._logger.event("land_finished", status="ok")

    def brake(self) -> None:
        """Command BRAKE mode as a safety action."""
        self._logger.event("brake_started")
        if self._cfg.dry_run:
            self._logger.command("brake", status="dry_run")
            self._logger.event("brake_finished", status="dry_run")
            return
        self._command_long(
            MAV_CMD_DO_SET_MODE,
            [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, _mode_number("BRAKE")],
            name="brake",
        )
        self._logger.event("brake_finished", status="ok")

    def loiter(self) -> None:
        """Command LOITER mode as a safety action."""
        self._logger.event("loiter_started")
        if self._cfg.dry_run:
            self._logger.command("loiter", status="dry_run")
            self._logger.event("loiter_finished", status="dry_run")
            return
        self._command_long(
            MAV_CMD_DO_SET_MODE,
            [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, _mode_number("LOITER")],
            name="loiter",
        )
        self._logger.event("loiter_finished", status="ok")

    def stop_motion(self) -> None:
        """Send one zero-velocity setpoint without propagating gate inhibition."""
        self._logger.event("stop_motion_started")
        try:
            self._send_body_velocity_unchecked(0.0, 0.0, 0.0, 0.0, name="stop_motion")
        except Exception as exc:
            self._logger.event("stop_motion_failed", error=repr(exc))
            return
        self._logger.event("stop_motion_finished", status="ok")

    def send_body_velocity(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float = 0.0,
    ) -> None:
        """Send one clamped body-frame velocity setpoint."""
        self._motion_gate.assert_motion_allowed()
        vx = _clamp(vx_mps, -self._cfg.limits.max_vx_mps, self._cfg.limits.max_vx_mps)
        vy = _clamp(vy_mps, -self._cfg.limits.max_vy_mps, self._cfg.limits.max_vy_mps)
        vz = _clamp(vz_mps, -self._cfg.limits.max_vz_mps, self._cfg.limits.max_vz_mps)
        yaw_rate = _clamp(
            yaw_rate_dps,
            -self._cfg.limits.max_yaw_rate_dps,
            self._cfg.limits.max_yaw_rate_dps,
        )
        self._send_body_velocity_unchecked(vx, vy, vz, yaw_rate)

    def _send_body_velocity_unchecked(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
        *,
        name: str = "body_velocity_setpoint",
    ) -> None:
        self._commands.send_body_velocity_setpoint(
            vx_mps,
            vy_mps,
            vz_mps,
            yaw_rate_dps,
            name=name,
        )

    def _command_long(self, command: int, params=(), *, name: str) -> None:
        try:
            self._commands.command_long(command, params, name=name)
        except MavlinkCommandTimeout as exc:
            raise MovementTimeout(str(exc)) from exc
        except MavlinkCommandRejected as exc:
            raise MovementRejected(str(exc)) from exc
        except MavlinkCommandError as exc:
            raise MovementError(str(exc)) from exc

    def _wait_for(self, predicate: Callable[[VehicleState], bool], description: str) -> None:
        deadline = time.monotonic() + self._cfg.mavlink.command_timeout_s
        while time.monotonic() <= deadline:
            if predicate(self._state_provider()):
                return
            time.sleep(0.02)
        raise MovementTimeout(f"Timed out waiting for {description}")


def _normalize_mode(mode: str | None) -> str:
    if mode is None:
        return ""
    return mode.upper()


def _mode_number(mode: str) -> int:
    try:
        return COPTER_MODES[mode]
    except KeyError as exc:
        raise MovementRejected(f"Unsupported mode: {mode}") from exc


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))
