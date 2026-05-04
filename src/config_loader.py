"""Typed configuration loading for Cooper_drone."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


VALID_FAILSAFE_ACTIONS = {"land", "brake", "loiter", "rtl", "stop", "force_disarm", "none"}

# force_disarm may only appear in kill_action.
FORCE_DISARM_ALLOWED_KEYS = {"kill_action"}

# Keys that must never be force_disarm or disarm.
NON_KILL_ACTION_KEYS = {
    "failsafe_action",
    "link_lost_action",
    "revoke_action",
    "rc_stale_action",
    "exit_action",
    "standby_exit_action",
}
SAFETY_ACTION_KEYS = (
    "failsafe_action",
    "kill_action",
    "link_lost_action",
    "revoke_action",
    "rc_stale_action",
    "exit_action",
    "standby_exit_action",
)


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
    kill_action: str = "land"
    link_lost_action: str = "land"
    revoke_action: str = "loiter"
    rc_stale_action: str = "brake"
    allow_force_disarm_on_kill: bool = False
    action_retry_interval_s: float = 1.0
    action_log_throttle_s: float = 1.0
    poll_hz: float = 10.0
    exit_action: str = "land"
    standby_exit_action: str = "none"


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
        safety=_build_safety_config(_required_mapping(raw, "safety")),
        logging=LoggingConfig(**_required_mapping(raw, "logging")),
        vision=VisionConfig(**_required_mapping(raw, "vision")),
    )


def _build_safety_config(raw: dict[str, Any]) -> SafetyConfig:
    safety_raw = dict(raw)
    for key in SAFETY_ACTION_KEYS:
        if key in safety_raw:
            value = safety_raw[key]
            if not isinstance(value, str):
                raise ConfigError(f"Configuration key safety.{key} must be str")
            safety_raw[key] = value.lower()
    return SafetyConfig(**safety_raw)


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
    for action_key in SAFETY_ACTION_KEYS:
        action = getattr(cfg.safety, action_key)
        if action not in VALID_FAILSAFE_ACTIONS:
            allowed = ", ".join(sorted(VALID_FAILSAFE_ACTIONS))
            raise ConfigError(f"Invalid {action_key}: {action}. Allowed: {allowed}")

    _validate_force_disarm_constraints(cfg)

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
    _require_positive("safety.poll_hz", cfg.safety.poll_hz)
    _require_non_negative("safety.action_retry_interval_s", cfg.safety.action_retry_interval_s)
    _require_non_negative("safety.action_log_throttle_s", cfg.safety.action_log_throttle_s)
    _require_bool("safety.allow_force_disarm_on_kill", cfg.safety.allow_force_disarm_on_kill)

    if not cfg.dry_run and cfg.mavlink.connection_string == "dry-run":
        raise ConfigError("connection_string dry-run is only allowed when dry_run is true")


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ConfigError(f"Configuration value {name} must be greater than 0")


def _validate_force_disarm_constraints(cfg: AppConfig) -> None:
    safety = cfg.safety

    for key in NON_KILL_ACTION_KEYS:
        action = getattr(safety, key, "land")
        if action in {"force_disarm", "disarm"}:
            raise ConfigError(
                f"safety.{key} cannot be '{action}'. "
                "force_disarm is only allowed for kill_action."
            )

    if safety.kill_action == "force_disarm" and not safety.allow_force_disarm_on_kill:
        raise ConfigError(
            "safety.kill_action=force_disarm requires "
            "safety.allow_force_disarm_on_kill=true"
        )


def _require_non_negative(name: str, value: float) -> None:
    if value < 0:
        raise ConfigError(f"Configuration value {name} must be greater than or equal to 0")


def _require_bool(name: str, value: bool) -> None:
    if not isinstance(value, bool):
        raise ConfigError(f"Configuration value {name} must be bool")
