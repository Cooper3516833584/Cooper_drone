"""
types — 共享数据结构

所有模块间传递的轻量级数据容器定义在此处。
使用 ``dataclass`` 保持轻量，避免外部依赖。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class VehicleSnapshot:
    """飞控遥测快照——由 TelemetryHub 采样并发布。

    Attributes:
        timestamp:    采样 UNIX 时间戳（time.time()）
        mode:         当前飞行模式，如 ``"GUIDED"`` / ``"LOITER"``
        armed:        是否已解锁
        lat:          纬度（°）
        lon:          经度（°）
        alt_rel_m:    相对起飞点高度（米）
        alt_msl_m:    海拔高度（米）
        vx:           NED 北向速度（m/s）
        vy:           NED 东向速度（m/s）
        vz:           NED 下向速度（m/s，正值向下）
        heading_deg:  航向角（°，0-360）
        battery_v:    电池电压（V）
        battery_pct:  电池百分比（0-100，-1 表示不可用）
        gps_fix:      GPS Fix 类型（0=无，3=3D Fix）
        gps_num_sat:  可见卫星数
        ekf_ok:       EKF 状态是否正常
        rc_channels:  RC 通道值字典，键为通道号（1-based），值为 PWM
    """
    timestamp: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    lat: float = 0.0
    lon: float = 0.0
    alt_rel_m: float = 0.0
    alt_msl_m: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    heading_deg: float = 0.0
    battery_v: float = 0.0
    battery_pct: float = -1.0
    gps_fix: int = 0
    gps_num_sat: int = 0
    ekf_ok: bool = False
    rc_channels: dict[int, int] = field(default_factory=dict)


@dataclass
class VisionTarget:
    """视觉目标检测结果——由 Tracker 产出，供 Mission 消费。

    Attributes:
        timestamp:    检测时间戳（time.time()）
        dx:           目标在图像中相对画面中心的水平偏移（归一化 -1.0 ~ 1.0）
        dy:           目标在图像中相对画面中心的垂直偏移（归一化 -1.0 ~ 1.0）
        confidence:   检测置信度（0.0 ~ 1.0）
        detected:     本帧是否检出目标
    """
    timestamp: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    confidence: float = 0.0
    detected: bool = False

    def is_stale(self, max_age_s: float = 0.5) -> bool:
        """判断目标是否过期。

        Args:
            max_age_s: 最大允许年龄（秒），默认 0.5s。

        Returns:
            True 表示目标已过期，不应再使用。
        """
        return (time.time() - self.timestamp) > max_age_s
