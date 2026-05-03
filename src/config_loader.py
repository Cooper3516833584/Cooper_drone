"""Typed configuration loading for Cooper_drone."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


VALID_FAILSAFE_ACTIONS = {"land", "brake", "loiter", "disarm", "none"}


class ConfigError(ValueError):
    """Raised when a configuration file is missing required data or is invalid."""


@dataclass
class MavlinkConfig:
    """MAVLink connection and command settings."""

    connection_string: str
    baud: int
    source_system: int
    source_component: int
    target_system: int | None
    target_component: int | None
    heartbeat_timeout_s: float
    command_timeout_s: float
    command_retries: int


@dataclass
class LimitsConfig:
    """Flight command limits used before any movement command is sent."""

    max_vx_mps: float
    max_vy_mps: float
    max_vz_mps: float
    max_yaw_rate_dps: float
    max_altitude_m: float
    takeoff_altitude_tolerance_m: float


@dataclass
class SafetyConfig:
    """Safety gate behavior and timeout settings."""

    enabled: bool
    allow_arm: bool
    require_guided: bool
    require_heartbeat: bool
    heartbeat_loss_timeout_s: float
    rc_stale_timeout_s: float
    manual_takeover_enabled: bool
    kill_switch_enabled: bool
    failsafe_action: str


@dataclass
class LoggingConfig:
    """Logging settings for console and telemetry logs."""

    log_dir: str
    telemetry_log_hz: float
    jsonl_enabled: bool
    console_level: str


@dataclass
class VisionConfig:
    """Vision input settings."""

    enabled: bool
    camera_index: int
    width: int
    height: int
    fps: int
    dry_run_source: str | None


@dataclass
class AppConfig:
    """Resolved application configuration."""

    profile_name: str
    dry_run: bool
    mavlink: MavlinkConfig
    limits: LimitsConfig
    safety: SafetyConfig
    logging: LoggingConfig
    vision: VisionConfig


def load_config(path: str | Path, *, dry_run_override: bool | None = None) -> AppConfig:
    """Load, merge, and validate an application configuration file."""
    config_path = Path(path)
    raw = _load_with_defaults(config_path)

    if dry_run_override is not None:
        raw["dry_run"] = dry_run_override

    cfg = _build_config(raw)
    _validate_config(cfg)
    return cfg


def dump_resolved_config(cfg: AppConfig) -> dict[str, Any]:
    """Return a JSON-serializable dictionary for a resolved configuration."""
    return asdict(cfg)


def _load_with_defaults(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        raise ConfigError(f"Configuration file does not exist: {config_path}")

    data = _read_yaml_mapping(config_path)
    default_path = config_path.parent / "default.yaml"

    if config_path.resolve() == default_path.resolve() or not default_path.exists():
        return data

    defaults = _read_yaml_mapping(default_path)
    return _deep_merge(defaults, data)


def _read_yaml_mapping(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}

    if not isinstance(loaded, dict):
        raise ConfigError(f"Configuration root must be a mapping: {path}")

    return loaded


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _build_config(raw: dict[str, Any]) -> AppConfig:
    return AppConfig(
        profile_name=_required(raw, "profile_name", str),
        dry_run=_required(raw, "dry_run", bool),
        mavlink=MavlinkConfig(**_required_mapping(raw, "mavlink")),
        limits=LimitsConfig(**_required_mapping(raw, "limits")),
        safety=SafetyConfig(**_required_mapping(raw, "safety")),
        logging=LoggingConfig(**_required_mapping(raw, "logging")),
        vision=VisionConfig(**_required_mapping(raw, "vision")),
    )


def _required(raw: dict[str, Any], key: str, expected_type: type) -> Any:
    if key not in raw:
        raise ConfigError(f"Missing required configuration key: {key}")
    value = raw[key]
    if not isinstance(value, expected_type):
        raise ConfigError(f"Configuration key {key} must be {expected_type.__name__}")
    return value


def _required_mapping(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ConfigError(f"Configuration key {key} must be a mapping")
    return value


def _validate_config(cfg: AppConfig) -> None:
    if cfg.safety.failsafe_action not in VALID_FAILSAFE_ACTIONS:
        allowed = ", ".join(sorted(VALID_FAILSAFE_ACTIONS))
        raise ConfigError(f"Invalid failsafe_action: {cfg.safety.failsafe_action}. Allowed: {allowed}")

    _require_positive("mavlink.heartbeat_timeout_s", cfg.mavlink.heartbeat_timeout_s)
    _require_positive("mavlink.command_timeout_s", cfg.mavlink.command_timeout_s)
    _require_positive("safety.heartbeat_loss_timeout_s", cfg.safety.heartbeat_loss_timeout_s)
    _require_positive("safety.rc_stale_timeout_s", cfg.safety.rc_stale_timeout_s)

    _require_positive("limits.max_vx_mps", cfg.limits.max_vx_mps)
    _require_positive("limits.max_vy_mps", cfg.limits.max_vy_mps)
    _require_positive("limits.max_vz_mps", cfg.limits.max_vz_mps)
    _require_positive("limits.max_yaw_rate_dps", cfg.limits.max_yaw_rate_dps)
    _require_positive("limits.max_altitude_m", cfg.limits.max_altitude_m)
    _require_positive("limits.takeoff_altitude_tolerance_m", cfg.limits.takeoff_altitude_tolerance_m)

    if not cfg.dry_run and cfg.mavlink.connection_string == "dry-run":
        raise ConfigError("connection_string dry-run is only allowed when dry_run is true")


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ConfigError(f"Configuration value {name} must be greater than 0")
