"""
config_loader — 配置统一入口

职责：
    从 config/vehicle.yaml 加载所有常量，并支持环境变量覆盖。
    所有常量不允许散落在代码中，必须通过本模块读取。

用法：
    cfg = load_config("config/vehicle.yaml")
    print(cfg.mavlink.port)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from dotenv import load_dotenv


# ── 配置数据类 ────────────────────────────────────────────


@dataclass(frozen=True)
class MavlinkConfig:
    """MAVLink 串口连接参数。"""
    port: str = "/dev/ttyAMA0"
    baud: int = 115200
    connect_timeout_s: int = 30
    retry_interval_s: int = 5
    max_retries: int = 5


@dataclass(frozen=True)
class RcConfig:
    """RC 遥控器通道映射。"""
    mode_channel: int = 5
    kill_channel: int = 8
    guided_pwm_min: int = 1700
    kill_pwm_min: int = 1700


@dataclass(frozen=True)
class LimitsConfig:
    """飞行安全限幅。"""
    takeoff_alt_m: float = 2.0
    max_xy_vel_mps: float = 2.0
    max_z_vel_mps: float = 1.0
    max_yaw_rate_dps: float = 45.0
    position_tolerance_m: float = 0.5
    altitude_tolerance_m: float = 0.3


@dataclass(frozen=True)
class FailsafeConfig:
    """断链 / 失联安全策略。"""
    link_lost_action: str = "LAND"
    heartbeat_timeout_s: float = 3.0


@dataclass(frozen=True)
class StreamConfig:
    """Flask MJPEG 推流参数。"""
    enabled: bool = False
    host: str = "0.0.0.0"
    port: int = 5000
    fps: int = 15


@dataclass(frozen=True)
class VisionConfig:
    """视觉子系统参数。"""
    camera_index_or_path: int | str = 0
    frame_width: int = 640
    frame_height: int = 480
    stream: StreamConfig = field(default_factory=StreamConfig)


@dataclass(frozen=True)
class TelemetryConfig:
    """遥测采样参数。"""
    sample_rate_hz: int = 5


@dataclass(frozen=True)
class LoggingConfig:
    """日志参数。"""
    level: str = "INFO"
    dir: str = "logs"
    max_bytes: int = 10_485_760   # 10 MB
    backup_count: int = 5


@dataclass(frozen=True)
class AppConfig:
    """应用顶层配置——聚合所有子配置。

    Attributes:
        mavlink:    MAVLink 串口连接参数
        rc:         RC 通道映射
        limits:     飞行安全限幅
        failsafe:   断链安全策略
        vision:     视觉子系统参数
        telemetry:  遥测采样参数
        logging:    日志参数
    """
    mavlink: MavlinkConfig = field(default_factory=MavlinkConfig)
    rc: RcConfig = field(default_factory=RcConfig)
    limits: LimitsConfig = field(default_factory=LimitsConfig)
    failsafe: FailsafeConfig = field(default_factory=FailsafeConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


# ── 辅助函数 ──────────────────────────────────────────────


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """递归合并两个字典，*override* 的值优先。"""
    merged: dict[str, Any] = dict(base)
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _env_overrides() -> dict[str, Any]:
    """读取环境变量，构建与 YAML 同结构的覆盖字典。

    映射规则（参见 .env.example）：
        MAVLINK_PORT  -> mavlink.port
        MAVLINK_BAUD  -> mavlink.baud
        RC_MODE_CHANNEL -> rc.mode_channel
        ...
    """
    overrides: dict[str, Any] = {}

    def _set_nested(d: dict[str, Any], keys: list[str], value: Any) -> None:
        for k in keys[:-1]:
            d = d.setdefault(k, {})
        d[keys[-1]] = value

    env_map: list[tuple[str, list[str], type]] = [
        ("MAVLINK_PORT",     ["mavlink", "port"],            str),
        ("MAVLINK_BAUD",     ["mavlink", "baud"],            int),
        ("RC_MODE_CHANNEL",  ["rc", "mode_channel"],         int),
        ("RC_KILL_CHANNEL",  ["rc", "kill_channel"],         int),
        ("LOG_LEVEL",        ["logging", "level"],           str),
        ("LOG_DIR",          ["logging", "dir"],             str),
        ("STREAM_HOST",      ["vision", "stream", "host"],   str),
        ("STREAM_PORT",      ["vision", "stream", "port"],   int),
        ("STREAM_FPS",       ["vision", "stream", "fps"],    int),
        ("CAMERA_INDEX",     ["vision", "camera_index_or_path"], int),
    ]

    for env_key, yaml_path, conv in env_map:
        val = os.environ.get(env_key)
        if val is not None:
            try:
                _set_nested(overrides, yaml_path, conv(val))
            except (ValueError, TypeError):
                pass  # 跳过无法转换的值

    return overrides


def _build_sub(cls: type, data: dict[str, Any] | None) -> Any:
    """通用子配置构建：如果 data 为 None 返回默认实例。"""
    if data is None:
        return cls()
    # 特殊处理嵌套子配置（如 VisionConfig.stream）
    if cls is VisionConfig and "stream" in data:
        data = dict(data)
        data["stream"] = StreamConfig(**data["stream"])
    return cls(**data)


# ── 公开 API ─────────────────────────────────────────────


def load_config(path: str = "config/vehicle.yaml") -> AppConfig:
    """加载并返回 ``AppConfig`` 实例。

    加载优先级（高覆盖低）：
        1. YAML 文件默认值
        2. 环境变量覆盖（需 ``.env`` 或系统环境变量）

    Args:
        path: YAML 配置文件路径。

    Returns:
        AppConfig: 冻结（frozen）的全局配置实例。

    Raises:
        FileNotFoundError: 配置文件不存在时抛出。
        yaml.YAMLError: YAML 格式错误时抛出。
    """
    # 加载 .env（如果存在）
    dotenv_path = Path(path).resolve().parent.parent / ".env"
    if dotenv_path.exists():
        load_dotenv(dotenv_path)

    # 读取 YAML
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path.resolve()}")

    with open(config_path, "r", encoding="utf-8") as f:
        raw: dict[str, Any] = yaml.safe_load(f) or {}

    # 环境变量覆盖
    env = _env_overrides()
    merged = _deep_merge(raw, env)

    # 构建并返回
    return AppConfig(
        mavlink=_build_sub(MavlinkConfig, merged.get("mavlink")),
        rc=_build_sub(RcConfig, merged.get("rc")),
        limits=_build_sub(LimitsConfig, merged.get("limits")),
        failsafe=_build_sub(FailsafeConfig, merged.get("failsafe")),
        vision=_build_sub(VisionConfig, merged.get("vision")),
        telemetry=_build_sub(TelemetryConfig, merged.get("telemetry")),
        logging=_build_sub(LoggingConfig, merged.get("logging")),
    )
