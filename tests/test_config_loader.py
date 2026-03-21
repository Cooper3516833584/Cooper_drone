from __future__ import annotations

from pathlib import Path

import pytest

from src.config_loader import ConfigError, load_config

DATA = Path(__file__).parent / "data"


def test_yaml_load_and_defaults() -> None:
    cfg = load_config(DATA / "cfg_empty.yaml")
    assert cfg.mavlink.port == "/dev/serial0"
    assert cfg.telemetry.sample_hz == 5.0
    assert cfg.rc.stale_timeout_s == 1.5
    assert cfg.limits.max_xy_vel_mps > 0


def test_bool_string_parse() -> None:
    cfg = load_config(DATA / "cfg_bool_false.yaml")
    assert cfg.vision.enabled is False
    assert cfg.safety.check_home_position is False


def test_env_override(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("COOPER_MAVLINK_PORT", "udp:127.0.0.1:14550")
    monkeypatch.setenv("COOPER_MAVLINK_BAUD", "115200")
    cfg = load_config(DATA / "cfg_empty.yaml")
    assert cfg.mavlink.port == "udp:127.0.0.1:14550"
    assert cfg.mavlink.baud == 115200


def test_invalid_config_raise() -> None:
    with pytest.raises(ConfigError):
        load_config(DATA / "cfg_invalid.yaml")

