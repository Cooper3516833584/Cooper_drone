"""Tests for VSCode project configuration."""

from __future__ import annotations

import json
from pathlib import Path


def test_launch_json_is_valid_json() -> None:
    """Parse VSCode launch configuration as JSON."""
    data = json.loads(Path(".vscode/launch.json").read_text(encoding="utf-8"))

    names = {item["name"] for item in data["configurations"]}
    assert "Cooper: healthcheck dry-run" in names
    assert "Cooper: current Python file" in names


def test_settings_json_is_valid_json() -> None:
    """Parse VSCode settings as JSON."""
    data = json.loads(Path(".vscode/settings.json").read_text(encoding="utf-8"))

    assert data["python.testing.pytestEnabled"] is True
    assert data["python.testing.pytestArgs"] == ["test"]
    assert data["python.analysis.extraPaths"] == ["${workspaceFolder}"]


def test_readme_contains_vscode_run_instructions() -> None:
    """Verify README documents VSCode one-click run flow."""
    readme = Path("README.md").read_text(encoding="utf-8")

    assert "## VSCode 一键运行" in readme
    assert "Cooper: healthcheck dry-run" in readme
    assert "PYTHONPATH=${workspaceFolder}" in readme
