"""Application context assembly for root task scripts."""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Callable

from src.config_loader import AppConfig, load_config
from src.logging_runtime import RuntimeLogger, setup_runtime_logging
from src.mavlink_commands import MavlinkCommandService
from src.mavlink_connection import MavlinkConnection
from src.mission_runtime import CancellationToken
from src.motion_gate import MotionGate
from src.movement import DroneMovement
from src.safety import SafetySupervisor
from src.safety_actions import SafetyActionExecutor
from src.state import VehicleState


@dataclass
class AppContext:
    """Runtime dependencies shared by root task scripts."""

    cfg: AppConfig
    logger: RuntimeLogger
    connection: MavlinkConnection | None
    commands: MavlinkCommandService
    motion_gate: MotionGate
    movement: DroneMovement
    token: CancellationToken
    safety: SafetySupervisor | None
    state_provider: Callable[[], VehicleState]

    def close(self) -> None:
        """Stop background services and close runtime resources."""
        if self.safety is not None:
            self.safety.stop()
            self.safety = None
        if self.connection is not None:
            self.connection.close()
            self.connection = None
        self.logger.close()


def build_app_context(config_path: str, *, dry_run: bool | None = None) -> AppContext:
    """Build the application context for a root task script."""
    cfg = load_config(config_path, dry_run_override=dry_run)
    if cfg.dry_run:
        cfg = _prepare_dry_run_config(cfg)

    logger = setup_runtime_logging(cfg)
    connection = MavlinkConnection(cfg.mavlink, logger)

    if cfg.dry_run:
        state_provider = _dry_run_state_provider()
        logger.event("app_context_dry_run_connection_skipped")
    else:
        connection.connect()
        connection.wait_heartbeat()
        connection.start_receiver()
        state_provider = connection.state_snapshot

    commands = MavlinkCommandService(connection, logger)
    motion_gate = MotionGate()
    token = CancellationToken()
    movement = DroneMovement(cfg, commands, state_provider, motion_gate, logger)
    safety = None

    if cfg.safety.enabled:
        safety_actions = SafetyActionExecutor(
            movement,
            logger,
            allow_force_disarm=cfg.safety.allow_force_disarm_on_kill,
        )
        safety = SafetySupervisor(
            state_provider,
            cfg.safety,
            motion_gate,
            logger,
            token.cancel,
            action_executor=safety_actions,
            check_interval_s=1.0 / cfg.safety.poll_hz,
        )
        safety.start()

    return AppContext(
        cfg=cfg,
        logger=logger,
        connection=connection,
        commands=commands,
        motion_gate=motion_gate,
        movement=movement,
        token=token,
        safety=safety,
        state_provider=state_provider,
    )


def _prepare_dry_run_config(cfg: AppConfig) -> AppConfig:
    return replace(
        cfg,
        safety=replace(cfg.safety, allow_arm=True),
    )


def _dry_run_state_provider():
    def state_provider() -> VehicleState:
        now = time.time()
        return VehicleState(
            last_heartbeat_ts=now,
            armed=True,
            mode="GUIDED",
            system_status="dry_run",
            relative_alt_m=0.0,
            rc_last_update_ts=now,
        )

    return state_provider
