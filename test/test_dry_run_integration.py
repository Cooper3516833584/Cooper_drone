"""Dry-run integration tests for root task scripts."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_healthcheck_dry_run_integration(dry_run_config_path: Path) -> None:
    """Run the healthcheck task in dry-run mode without hardware."""
    result = _run(["task_healthcheck.py", "--config", str(dry_run_config_path), "--dry-run"])

    assert result.returncode == 0, result.stdout + result.stderr
    assert "dry-run: skipping MAVLink connection" in result.stdout


def test_takeoff_hover_dry_run_integration(dry_run_config_path: Path) -> None:
    """Run takeoff hover in dry-run mode without hardware."""
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
    assert "takeoff hover finished" in result.stdout


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, *args],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )
