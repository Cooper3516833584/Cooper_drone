from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from src.types import FailsafeAction


class ConfigError(ValueError):
    """配置解析错误。"""


@dataclass(frozen=True)
class RequestRatesConfig:
    heartbeat_hz: float = 1.0
    global_position_int_hz: float = 10.0
    sys_status_hz: float = 2.0
    gps_raw_int_hz: float = 5.0
    rc_channels_hz: float = 10.0
    home_position_hz: float = 1.0


@dataclass(frozen=True)
class MavlinkConfig:
    port: str = "/dev/serial0"
    baud: int = 921600
    connect_timeout_s: float = 10.0
    retry_interval_s: float = 3.0
    max_retries: int = 0
    heartbeat_timeout_s: float = 3.0
    wait_heartbeat_timeout_s: float = 10.0
    source_system: int = 245
    source_component: int = 191
    command_ack_timeout_s: float = 2.0
    command_retries: int = 3
    request_rates: RequestRatesConfig = field(default_factory=RequestRatesConfig)


@dataclass(frozen=True)
class RcConfig:
    mode_channel: int = 5
    kill_channel: int = 8
    kill_channel_aux: int = 7
    guided_pwm_threshold: int = 1500
    kill_pwm_threshold: int = 1800
    stale_timeout_s: float = 1.5


@dataclass(frozen=True)
class LimitsConfig:
    takeoff_alt_m: float = 2.0
    hover_seconds: float = 5.0
    max_xy_vel_mps: float = 1.0
    max_z_vel_mps: float = 0.5
    max_yaw_rate_dps: float = 30.0
    goto_accept_radius_m: float = 1.0
    altitude_accept_error_m: float = 0.5
    # 额外安全阈值：用于 preflight 校验。
    min_gps_fix_type: int = 3
    min_battery_percent: int = 20
    min_battery_voltage_v: float = 14.0


@dataclass(frozen=True)
class FailsafeConfig:
    link_lost_action: FailsafeAction = FailsafeAction.LAND
    revoke_action: FailsafeAction = FailsafeAction.LOITER


@dataclass(frozen=True)
class TelemetryConfig:
    sample_hz: float = 5.0


@dataclass(frozen=True)
class SafetyConfig:
    poll_hz: float = 10.0
    check_home_position: bool = False


@dataclass(frozen=True)
class VisionConfig:
    enabled: bool = False
    stream_enabled: bool = False
    stream_host: str = "0.0.0.0"
    stream_port: int = 5000


@dataclass(frozen=True)
class AppRuntimeConfig:
    log_level: str = "INFO"


@dataclass(frozen=True)
class AppConfig:
    mavlink: MavlinkConfig = field(default_factory=MavlinkConfig)
    rc: RcConfig = field(default_factory=RcConfig)
    limits: LimitsConfig = field(default_factory=LimitsConfig)
    failsafe: FailsafeConfig = field(default_factory=FailsafeConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    app: AppRuntimeConfig = field(default_factory=AppRuntimeConfig)


_TRUE_VALUES = {"1", "true", "yes", "on", "y", "t"}
_FALSE_VALUES = {"0", "false", "no", "off", "n", "f"}
_ALLOWED_FAILSAFE_ACTIONS = {action.value for action in FailsafeAction}


def _expect_mapping(value: Any, path: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ConfigError(f"{path} 必须是对象/字典，当前类型: {type(value).__name__}")
    return value


def _to_str(value: Any, path: str, default: str) -> str:
    if value is None:
        return default
    if isinstance(value, str):
        text = value.strip()
        if text:
            return text
        raise ConfigError(f"{path} 不能为空字符串")
    raise ConfigError(f"{path} 必须是字符串，当前类型: {type(value).__name__}")


def _to_bool(value: Any, path: str, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in _TRUE_VALUES:
            return True
        if normalized in _FALSE_VALUES:
            return False
    raise ConfigError(f"{path} 不是合法布尔值: {value!r}")


def _to_int(value: Any, path: str, default: int, min_value: int | None = None) -> int:
    if value is None:
        number = default
    elif isinstance(value, bool):
        raise ConfigError(f"{path} 不能是布尔值")
    elif isinstance(value, int):
        number = value
    elif isinstance(value, str):
        try:
            number = int(value.strip())
        except ValueError as exc:
            raise ConfigError(f"{path} 不是合法整数: {value!r}") from exc
    else:
        raise ConfigError(f"{path} 必须是整数，当前类型: {type(value).__name__}")
    if min_value is not None and number < min_value:
        raise ConfigError(f"{path} 不能小于 {min_value}")
    return number


def _to_float(value: Any, path: str, default: float, min_value: float | None = None) -> float:
    if value is None:
        number = default
    elif isinstance(value, bool):
        raise ConfigError(f"{path} 不能是布尔值")
    elif isinstance(value, (int, float)):
        number = float(value)
    elif isinstance(value, str):
        try:
            number = float(value.strip())
        except ValueError as exc:
            raise ConfigError(f"{path} 不是合法数字: {value!r}") from exc
    else:
        raise ConfigError(f"{path} 必须是数字，当前类型: {type(value).__name__}")
    if min_value is not None and number < min_value:
        raise ConfigError(f"{path} 不能小于 {min_value}")
    return number


def _to_failsafe_action(value: Any, path: str, default: FailsafeAction) -> FailsafeAction:
    text = _to_str(value, path, default.value).upper()
    if text not in _ALLOWED_FAILSAFE_ACTIONS:
        allowed = ", ".join(sorted(_ALLOWED_FAILSAFE_ACTIONS))
        raise ConfigError(f"{path} 必须是以下之一: {allowed}，当前值: {text}")
    return FailsafeAction(text)


def _build_request_rates(raw: dict[str, Any]) -> RequestRatesConfig:
    return RequestRatesConfig(
        heartbeat_hz=_to_float(raw.get("heartbeat_hz"), "mavlink.request_rates.heartbeat_hz", 1.0, min_value=0.1),
        global_position_int_hz=_to_float(
            raw.get("global_position_int_hz"),
            "mavlink.request_rates.global_position_int_hz",
            10.0,
            min_value=0.1,
        ),
        sys_status_hz=_to_float(raw.get("sys_status_hz"), "mavlink.request_rates.sys_status_hz", 2.0, min_value=0.1),
        gps_raw_int_hz=_to_float(raw.get("gps_raw_int_hz"), "mavlink.request_rates.gps_raw_int_hz", 5.0, min_value=0.1),
        rc_channels_hz=_to_float(raw.get("rc_channels_hz"), "mavlink.request_rates.rc_channels_hz", 10.0, min_value=0.1),
        home_position_hz=_to_float(raw.get("home_position_hz"), "mavlink.request_rates.home_position_hz", 1.0, min_value=0.1),
    )


def _build_mavlink(raw: dict[str, Any]) -> MavlinkConfig:
    rates_raw = _expect_mapping(raw.get("request_rates"), "mavlink.request_rates")
    return MavlinkConfig(
        port=_to_str(raw.get("port"), "mavlink.port", "/dev/serial0"),
        baud=_to_int(raw.get("baud"), "mavlink.baud", 921600, min_value=1),
        connect_timeout_s=_to_float(raw.get("connect_timeout_s"), "mavlink.connect_timeout_s", 10.0, min_value=0.1),
        retry_interval_s=_to_float(raw.get("retry_interval_s"), "mavlink.retry_interval_s", 3.0, min_value=0.1),
        max_retries=_to_int(raw.get("max_retries"), "mavlink.max_retries", 0, min_value=0),
        heartbeat_timeout_s=_to_float(raw.get("heartbeat_timeout_s"), "mavlink.heartbeat_timeout_s", 3.0, min_value=0.1),
        wait_heartbeat_timeout_s=_to_float(
            raw.get("wait_heartbeat_timeout_s"),
            "mavlink.wait_heartbeat_timeout_s",
            10.0,
            min_value=0.1,
        ),
        source_system=_to_int(raw.get("source_system"), "mavlink.source_system", 245, min_value=1),
        source_component=_to_int(raw.get("source_component"), "mavlink.source_component", 191, min_value=1),
        command_ack_timeout_s=_to_float(
            raw.get("command_ack_timeout_s"),
            "mavlink.command_ack_timeout_s",
            2.0,
            min_value=0.1,
        ),
        command_retries=_to_int(raw.get("command_retries"), "mavlink.command_retries", 3, min_value=0),
        request_rates=_build_request_rates(rates_raw),
    )


def _build_rc(raw: dict[str, Any]) -> RcConfig:
    return RcConfig(
        mode_channel=_to_int(raw.get("mode_channel"), "rc.mode_channel", 5, min_value=1),
        kill_channel=_to_int(raw.get("kill_channel"), "rc.kill_channel", 8, min_value=1),
        kill_channel_aux=_to_int(raw.get("kill_channel_aux"), "rc.kill_channel_aux", 7, min_value=1),
        guided_pwm_threshold=_to_int(raw.get("guided_pwm_threshold"), "rc.guided_pwm_threshold", 1500, min_value=1),
        kill_pwm_threshold=_to_int(raw.get("kill_pwm_threshold"), "rc.kill_pwm_threshold", 1800, min_value=1),
        stale_timeout_s=_to_float(raw.get("stale_timeout_s"), "rc.stale_timeout_s", 1.5, min_value=0.1),
    )


def _build_limits(raw: dict[str, Any]) -> LimitsConfig:
    return LimitsConfig(
        takeoff_alt_m=_to_float(raw.get("takeoff_alt_m"), "limits.takeoff_alt_m", 2.0, min_value=0.1),
        hover_seconds=_to_float(raw.get("hover_seconds"), "limits.hover_seconds", 5.0, min_value=0.0),
        max_xy_vel_mps=_to_float(raw.get("max_xy_vel_mps"), "limits.max_xy_vel_mps", 1.0, min_value=0.01),
        max_z_vel_mps=_to_float(raw.get("max_z_vel_mps"), "limits.max_z_vel_mps", 0.5, min_value=0.01),
        max_yaw_rate_dps=_to_float(raw.get("max_yaw_rate_dps"), "limits.max_yaw_rate_dps", 30.0, min_value=0.1),
        goto_accept_radius_m=_to_float(
            raw.get("goto_accept_radius_m"),
            "limits.goto_accept_radius_m",
            1.0,
            min_value=0.1,
        ),
        altitude_accept_error_m=_to_float(
            raw.get("altitude_accept_error_m"),
            "limits.altitude_accept_error_m",
            0.5,
            min_value=0.05,
        ),
        min_gps_fix_type=_to_int(raw.get("min_gps_fix_type"), "limits.min_gps_fix_type", 3, min_value=0),
        min_battery_percent=_to_int(raw.get("min_battery_percent"), "limits.min_battery_percent", 20, min_value=0),
        min_battery_voltage_v=_to_float(
            raw.get("min_battery_voltage_v"),
            "limits.min_battery_voltage_v",
            14.0,
            min_value=0.0,
        ),
    )


def _build_failsafe(raw: dict[str, Any]) -> FailsafeConfig:
    return FailsafeConfig(
        link_lost_action=_to_failsafe_action(raw.get("link_lost_action"), "failsafe.link_lost_action", FailsafeAction.LAND),
        revoke_action=_to_failsafe_action(raw.get("revoke_action"), "failsafe.revoke_action", FailsafeAction.LOITER),
    )


def _build_telemetry(raw: dict[str, Any]) -> TelemetryConfig:
    return TelemetryConfig(
        sample_hz=_to_float(raw.get("sample_hz"), "telemetry.sample_hz", 5.0, min_value=0.1),
    )


def _build_safety(raw: dict[str, Any]) -> SafetyConfig:
    return SafetyConfig(
        poll_hz=_to_float(raw.get("poll_hz"), "safety.poll_hz", 10.0, min_value=0.1),
        check_home_position=_to_bool(raw.get("check_home_position"), "safety.check_home_position", False),
    )


def _build_vision(raw: dict[str, Any]) -> VisionConfig:
    return VisionConfig(
        enabled=_to_bool(raw.get("enabled"), "vision.enabled", False),
        stream_enabled=_to_bool(raw.get("stream_enabled"), "vision.stream_enabled", False),
        stream_host=_to_str(raw.get("stream_host"), "vision.stream_host", "0.0.0.0"),
        stream_port=_to_int(raw.get("stream_port"), "vision.stream_port", 5000, min_value=1),
    )


def _build_app(raw: dict[str, Any]) -> AppRuntimeConfig:
    return AppRuntimeConfig(log_level=_to_str(raw.get("log_level"), "app.log_level", "INFO").upper())


def _apply_env_overrides(cfg: AppConfig) -> AppConfig:
    """支持少量环境变量覆盖，便于部署与测试。"""
    port = os.getenv("COOPER_MAVLINK_PORT")
    baud = os.getenv("COOPER_MAVLINK_BAUD")
    log_level = os.getenv("COOPER_LOG_LEVEL")

    mavlink = cfg.mavlink
    app = cfg.app

    if port:
        mavlink = MavlinkConfig(**{**mavlink.__dict__, "port": port})
    if baud:
        mavlink = MavlinkConfig(**{**mavlink.__dict__, "baud": _to_int(baud, "env.COOPER_MAVLINK_BAUD", mavlink.baud, min_value=1)})
    if log_level:
        app = AppRuntimeConfig(log_level=_to_str(log_level, "env.COOPER_LOG_LEVEL", app.log_level).upper())

    return AppConfig(
        mavlink=mavlink,
        rc=cfg.rc,
        limits=cfg.limits,
        failsafe=cfg.failsafe,
        telemetry=cfg.telemetry,
        safety=cfg.safety,
        vision=cfg.vision,
        app=app,
    )


def load_config(path: str | Path) -> AppConfig:
    cfg_path = Path(path)
    if not cfg_path.exists():
        raise ConfigError(f"配置文件不存在: {cfg_path}")

    try:
        raw = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"YAML 语法错误: {exc}") from exc

    root = _expect_mapping(raw, "root")
    cfg = AppConfig(
        mavlink=_build_mavlink(_expect_mapping(root.get("mavlink"), "mavlink")),
        rc=_build_rc(_expect_mapping(root.get("rc"), "rc")),
        limits=_build_limits(_expect_mapping(root.get("limits"), "limits")),
        failsafe=_build_failsafe(_expect_mapping(root.get("failsafe"), "failsafe")),
        telemetry=_build_telemetry(_expect_mapping(root.get("telemetry"), "telemetry")),
        safety=_build_safety(_expect_mapping(root.get("safety"), "safety")),
        vision=_build_vision(_expect_mapping(root.get("vision"), "vision")),
        app=_build_app(_expect_mapping(root.get("app"), "app")),
    )
    return _apply_env_overrides(cfg)
