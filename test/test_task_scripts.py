"""Tests for root task scripts."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


TASK_SCRIPTS = [
    "task_healthcheck.py",
    "task_takeoff_hover.py",
    "task_waypoint_square.py",
    "task_vision_follow.py",
]


def test_each_task_script_supports_help() -> None:
    """Verify every root task script supports --help."""
    for script in TASK_SCRIPTS:
        result = _run([script, "--help"])
        assert result.returncode == 0, result.stderr
        assert "usage:" in result.stdout.lower()


def test_healthcheck_dry_run_returns_zero(dry_run_config_path: Path) -> None:
    """Run healthcheck in dry-run mode."""
    result = _run(["task_healthcheck.py", "--config", str(dry_run_config_path), "--dry-run"])

    assert result.returncode == 0, result.stdout + result.stderr


def test_takeoff_hover_dry_run_returns_zero(dry_run_config_path: Path) -> None:
    """Run takeoff hover in dry-run mode."""
    result = _run(
        [
            "task_takeoff_hover.py",
            "--config",
            str(dry_run_config_path),
            "--dry-run",
            "--altitude",
            "1",
            "--duration",
            "0.1",
        ]
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_takeoff_hover_invalid_altitude_returns_nonzero(dry_run_config_path: Path) -> None:
    """Return nonzero when a root task hits a mission error."""
    result = _run(
        [
            "task_takeoff_hover.py",
            "--config",
            str(dry_run_config_path),
            "--dry-run",
            "--altitude",
            "999",
            "--duration",
            "0.1",
        ]
    )

    assert result.returncode != 0
    assert "takeoff hover failed" in result.stdout


def test_task_scripts_do_not_contain_forbidden_low_level_calls() -> None:
    """Keep root tasks free of low-level MAVLink calls."""
    forbidden = ["pymavlink", "command_long_send"]
    for script in TASK_SCRIPTS:
        source = Path(script).read_text(encoding="utf-8")
        for term in forbidden:
            assert term not in source


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, *args],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )
