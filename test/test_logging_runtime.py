"""Tests for runtime logging."""

from __future__ import annotations

import json
from pathlib import Path

import yaml


def test_creates_log_directory(runtime_logger) -> None:
    """Create a unique runtime log directory."""
    assert runtime_logger.run_dir.exists()
    assert runtime_logger.run_dir.is_dir()


def test_writes_events_jsonl(runtime_logger) -> None:
    """Write event records to events.jsonl."""
    runtime_logger.event("test_event", value=1)

    record = _read_first_jsonl(runtime_logger.run_dir / "events.jsonl")
    assert record["type"] == "event"
    assert record["name"] == "test_event"
    assert record["run_id"] == runtime_logger.run_id
    assert isinstance(record["ts"], float)


def test_writes_commands_jsonl(runtime_logger) -> None:
    """Write command records to commands.jsonl."""
    runtime_logger.command("test_command", command_id=10)

    record = _read_first_jsonl(runtime_logger.run_dir / "commands.jsonl")
    assert record["type"] == "command"
    assert record["name"] == "test_command"


def test_writes_safety_jsonl(runtime_logger) -> None:
    """Write safety records to safety.jsonl."""
    runtime_logger.safety("test_safety", state="NORMAL")

    record = _read_first_jsonl(runtime_logger.run_dir / "safety.jsonl")
    assert record["type"] == "safety"
    assert record["name"] == "test_safety"


def test_writes_telemetry_jsonl(runtime_logger) -> None:
    """Write telemetry records to telemetry.jsonl."""
    runtime_logger.telemetry(altitude_m=1.5)

    record = _read_first_jsonl(runtime_logger.run_dir / "telemetry.jsonl")
    assert record["type"] == "telemetry"
    assert record["name"] == "telemetry"
    assert record["altitude_m"] == 1.5


def test_files_are_readable_after_close(runtime_logger) -> None:
    """Keep log files readable after the logger is closed."""
    runtime_logger.event("close_readback")
    runtime_logger.close()

    content = (runtime_logger.run_dir / "events.jsonl").read_text(encoding="utf-8")
    assert "close_readback" in content


def test_resolved_config_yaml_is_saved(runtime_logger) -> None:
    """Save the resolved runtime configuration beside the logs."""
    config_path = runtime_logger.run_dir / "resolved_config.yaml"
    saved = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert saved["profile_name"] == "dry_run"
    assert saved["dry_run"] is True


def _read_first_jsonl(path: Path) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines
    return json.loads(lines[0])
