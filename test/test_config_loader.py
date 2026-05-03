"""Tests for the typed configuration loader."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from src.config_loader import ConfigError, dump_resolved_config, load_config


FIXTURE_DIR = Path("test/fixtures/config_loader")


def test_loads_dry_run_config() -> None:
    """Load the dry-run profile with inherited defaults."""
    cfg = load_config("config/dry_run.yaml")

    assert cfg.profile_name == "dry_run"
    assert cfg.dry_run is True
    assert cfg.mavlink.connection_string == "dry-run"
    assert cfg.safety.allow_arm is False
    assert cfg.safety.kill_action == "land"
    assert cfg.safety.link_lost_action == "land"
    assert cfg.safety.revoke_action == "loiter"
    assert cfg.safety.rc_stale_action == "brake"
    assert cfg.safety.allow_force_disarm_on_kill is False
    assert cfg.safety.action_retry_interval_s == 1.0


def test_dry_run_override_true_takes_effect() -> None:
    """Allow callers to force dry-run mode when loading another profile."""
    cfg = load_config("config/sitl.yaml", dry_run_override=True)

    assert cfg.dry_run is True


def test_invalid_failsafe_action_raises() -> None:
    """Reject unsupported failsafe actions."""
    with pytest.raises(ConfigError, match="Invalid failsafe_action"):
        load_config(FIXTURE_DIR / "invalid_failsafe.yaml")


def test_independent_safety_actions_parse_and_normalize() -> None:
    """Parse independent safety actions without reusing a legacy action."""
    cfg = load_config(FIXTURE_DIR / "custom_safety_actions.yaml")

    assert cfg.safety.kill_action == "land"
    assert cfg.safety.link_lost_action == "rtl"
    assert cfg.safety.revoke_action == "loiter"
    assert cfg.safety.rc_stale_action == "brake"
    assert cfg.safety.allow_force_disarm_on_kill is False


def test_invalid_independent_safety_action_raises() -> None:
    """Reject unsupported independent safety actions."""
    with pytest.raises(ConfigError, match="Invalid kill_action"):
        load_config(FIXTURE_DIR / "invalid_kill_action.yaml")


def test_invalid_force_disarm_bool_raises() -> None:
    """Reject non-boolean force-disarm configuration."""
    with pytest.raises(ConfigError, match="allow_force_disarm_on_kill"):
        load_config(FIXTURE_DIR / "invalid_force_disarm_bool.yaml")


def test_negative_timeout_raises() -> None:
    """Reject timeout values that are not positive."""
    with pytest.raises(ConfigError, match="must be greater than 0"):
        load_config(FIXTURE_DIR / "negative_timeout.yaml")


def test_dump_resolved_config_is_json_serializable() -> None:
    """Dump resolved config into a JSON-serializable dictionary."""
    cfg = load_config("config/dry_run.yaml")
    dumped = dump_resolved_config(cfg)

    json.dumps(dumped)
    assert dumped["profile_name"] == "dry_run"


def test_non_dry_run_rejects_dry_run_connection_string() -> None:
    """Reject dry-run connection strings when dry-run mode is disabled."""
    with pytest.raises(ConfigError, match="only allowed when dry_run is true"):
        load_config(FIXTURE_DIR / "non_dry_run_with_dry_run_connection.yaml")
