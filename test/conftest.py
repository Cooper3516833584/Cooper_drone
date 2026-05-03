"""Shared pytest fixtures for Cooper_drone tests."""

from __future__ import annotations

import logging
import re
import shutil
from dataclasses import replace
from pathlib import Path
from typing import Sequence
from uuid import uuid4

import pytest
import yaml

from src.config_loader import LoggingConfig, load_config
from src.logging_runtime import RuntimeLogger, setup_runtime_logging
from src.movement import (
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_NAV_TAKEOFF,
)
from src.state import VehicleState


@pytest.fixture
def tmp_path(request):
    """Provide a workspace-local tmp_path for systems with restricted temp dirs."""
    root = Path.cwd() / ".pytest_tmp"
    root.mkdir(exist_ok=True)
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.name)
    path = root / f"{safe_name}_{uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        resolved_root = root.resolve()
        resolved_path = path.resolve()
        if resolved_root in resolved_path.parents:
            shutil.rmtree(resolved_path, ignore_errors=True)


@pytest.fixture
def dry_run_cfg():
    """Return the resolved dry-run configuration."""
    return load_config("config/dry_run.yaml")


@pytest.fixture
def runtime_logger(tmp_path: Path, dry_run_cfg):
    """Create a runtime logger that writes only under pytest tmp_path."""
    test_cfg = replace(
        dry_run_cfg,
        logging=LoggingConfig(
            log_dir=str(tmp_path / "logs"),
            telemetry_log_hz=dry_run_cfg.logging.telemetry_log_hz,
            jsonl_enabled=dry_run_cfg.logging.jsonl_enabled,
            console_level=dry_run_cfg.logging.console_level,
        ),
    )
    logger = setup_runtime_logging(test_cfg)
    try:
        yield logger
    finally:
        logger.close()


@pytest.fixture
def fake_state_provider():
    """Return a mutable fake vehicle state provider."""
    state = VehicleState(mode="GUIDED", armed=True, relative_alt_m=0.0)

    def provider() -> VehicleState:
        return state

    provider.state = state
    return provider


@pytest.fixture
def fake_movement():
    """Return a fake movement object for solution and runtime tests."""
    return FakeMovement()


@pytest.fixture
def dry_run_config_path(tmp_path: Path) -> Path:
    """Create a dry-run config directory whose logs stay under tmp_path."""
    config_dir = tmp_path / "config"
    config_dir.mkdir()

    default_data = yaml.safe_load(Path("config/default.yaml").read_text(encoding="utf-8"))
    default_data["logging"]["log_dir"] = str(tmp_path / "logs")
    (config_dir / "default.yaml").write_text(yaml.safe_dump(default_data), encoding="utf-8")
    (config_dir / "dry_run.yaml").write_text(
        Path("config/dry_run.yaml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    return config_dir / "dry_run.yaml"


class FakeMovement:
    """Fake movement API shared by tests."""

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
    """Runtime logger that stores events in memory without file IO."""

    def __init__(self, run_dir: Path | None = None) -> None:
        """Create an in-memory runtime logger."""
        super().__init__(
            run_id="test-run",
            run_dir=run_dir or Path("unused-test-logs"),
            app_logger=logging.getLogger("test.memory_runtime_logger"),
        )
        self.records: list[dict] = []

    def event(self, name: str, **fields) -> None:
        """Capture an event record."""
        self.records.append({"type": "event", "name": name, "fields": fields})

    def command(self, name: str, **fields) -> None:
        """Capture a command record."""
        self.records.append({"type": "command", "name": name, "fields": fields})
