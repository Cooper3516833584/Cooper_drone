"""Tests for deployment documentation and helper scripts."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


DOCS = [
    "docs/sitl.md",
    "docs/hardware_setup.md",
    "docs/no_props_checklist.md",
    "docs/first_flight_checklist.md",
]


def test_deployment_docs_exist() -> None:
    """Verify deployment documentation files exist."""
    for path in DOCS:
        assert Path(path).exists()


def test_check_serial_link_supports_help() -> None:
    """Verify check_serial_link supports --help."""
    result = _run(["scripts/check_serial_link.py", "--help"])

    assert result.returncode == 0
    assert "usage:" in result.stdout.lower()


def test_print_config_supports_help() -> None:
    """Verify print_config supports --help."""
    result = _run(["scripts/print_config.py", "--help"])

    assert result.returncode == 0
    assert "usage:" in result.stdout.lower()


def test_print_config_dry_run_runs() -> None:
    """Verify print_config can resolve dry-run config."""
    result = _run(["scripts/print_config.py", "--config", "config/dry_run.yaml"])

    assert result.returncode == 0
    assert "profile_name: dry_run" in result.stdout


def test_check_serial_link_dry_run_skips_real_connection() -> None:
    """Verify check_serial_link skips real connection in dry-run."""
    result = _run(["scripts/check_serial_link.py", "--config", "config/dry_run.yaml", "--dry-run"])

    assert result.returncode == 0
    assert "dry-run: skipping MAVLink serial link check" in result.stdout


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, *args],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )
