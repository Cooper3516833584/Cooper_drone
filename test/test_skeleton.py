"""Skeleton tests for the initial project layout."""

from pathlib import Path


def test_required_project_files_exist() -> None:
    """Verify the initial project skeleton contains required files."""
    root = Path(__file__).resolve().parents[1]
    required_paths = [
        "README.md",
        "pyproject.toml",
        "config/default.yaml",
        "config/dry_run.yaml",
        "config/sitl.yaml",
        "config/stm32mp257_uart.yaml",
        "src/__init__.py",
        "src/Vision/__init__.py",
        "src/solutions/__init__.py",
        "task_healthcheck.py",
        "task_takeoff_hover.py",
        "task_waypoint_square.py",
        "task_vision_follow.py",
    ]

    missing = [path for path in required_paths if not (root / path).exists()]
    assert missing == []
