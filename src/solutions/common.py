"""Common helpers for high-level mission solutions."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

from src.config_loader import AppConfig
from src.logging_runtime import RuntimeLogger
from src.mission_runtime import CancellationToken
from src.movement import DroneMovement
from src.state import VehicleState


@dataclass
class SolutionContext:
    """Shared context passed into high-level mission solutions."""

    cfg: AppConfig
    movement: DroneMovement
    logger: RuntimeLogger
    token: CancellationToken
    state_provider: Callable[[], VehicleState]


def sleep_checked(token: CancellationToken, seconds: float, step_s: float = 0.1) -> None:
    """Sleep while periodically checking for mission cancellation."""
    if seconds < 0:
        raise ValueError("seconds must be greater than or equal to 0")
    if step_s <= 0:
        raise ValueError("step_s must be greater than 0")

    deadline = time.monotonic() + seconds
    while True:
        token.raise_if_cancelled()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(step_s, remaining))


def wait_until(
    token: CancellationToken,
    predicate: Callable[[], bool],
    timeout_s: float,
    step_s: float = 0.1,
) -> None:
    """Wait until a predicate is true while checking for mission cancellation."""
    if timeout_s <= 0:
        raise ValueError("timeout_s must be greater than 0")
    if step_s <= 0:
        raise ValueError("step_s must be greater than 0")

    deadline = time.monotonic() + timeout_s
    while True:
        token.raise_if_cancelled()
        if predicate():
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for solution condition")
        time.sleep(step_s)
