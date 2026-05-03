"""Tests for mission runtime helpers."""

from __future__ import annotations

import logging
from pathlib import Path

import pytest

from src.logging_runtime import RuntimeLogger
from src.mission_runtime import CancellationToken, MissionCancelled, run_mission


def test_cancellation_token_raises_after_cancel() -> None:
    """Raise after a cancellation token is cancelled."""
    token = CancellationToken()
    token.cancel("test cancel")

    with pytest.raises(MissionCancelled, match="test cancel"):
        token.raise_if_cancelled()


def test_run_mission_returns_nonzero_when_cancelled() -> None:
    """Return a clear nonzero code when a mission is cancelled."""
    events: list[tuple[str, dict]] = []
    logger = _memory_runtime_logger(events)

    def mission(token: CancellationToken) -> None:
        token.cancel("cancelled by test")

    result = run_mission("test_mission", mission, logger)

    assert result == 2
    assert any(name == "mission_cancelled" for name, _fields in events)


def test_run_mission_calls_teardown_on_exception() -> None:
    """Call teardown even if the mission raises."""
    events: list[tuple[str, dict]] = []
    teardown_called = False

    def mission(_token: CancellationToken) -> None:
        raise RuntimeError("mission error")

    def teardown() -> None:
        nonlocal teardown_called
        teardown_called = True

    result = run_mission("failing_mission", mission, _memory_runtime_logger(events), teardown)

    assert result == 1
    assert teardown_called is True
    assert any(name == "mission_failed" for name, _fields in events)


def _memory_runtime_logger(events: list[tuple[str, dict]]) -> RuntimeLogger:
    logger = logging.getLogger("test.mission")
    logger.addHandler(logging.NullHandler())
    runtime_logger = RuntimeLogger(
        run_id="test-run",
        run_dir=Path("logs/test-mission"),
        app_logger=logger,
    )

    def event(name: str, **fields):
        events.append((name, fields))

    runtime_logger.event = event
    return runtime_logger
