"""
test_config — 配置加载冒烟测试
"""

import os
import sys

# 确保项目根目录在 sys.path 中
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.config_loader import load_config


def test_load_default_config():
    """测试默认配置文件加载。"""
    cfg = load_config("config/vehicle.yaml")

    assert cfg.mavlink.port == "/dev/ttyAMA0"
    assert cfg.mavlink.baud == 115200
    assert cfg.rc.mode_channel == 5
    assert cfg.rc.kill_channel == 8
    assert cfg.limits.takeoff_alt_m == 2.0
    assert cfg.failsafe.link_lost_action == "LAND"
    assert cfg.vision.stream.port == 5000
    assert cfg.vision.stream.enabled is False
    assert cfg.logging.level == "INFO"

    # Fix 7: link_lost_timeout_s should not exist
    assert not hasattr(cfg.failsafe, "link_lost_timeout_s")

    print("[PASS] test_load_default_config")


def test_config_fields():
    """测试所有子配置字段可访问。"""
    cfg = load_config("config/vehicle.yaml")

    # MavlinkConfig
    _ = cfg.mavlink.connect_timeout_s
    _ = cfg.mavlink.retry_interval_s
    _ = cfg.mavlink.max_retries

    # RcConfig
    _ = cfg.rc.guided_pwm_min
    _ = cfg.rc.kill_pwm_min

    # LimitsConfig
    _ = cfg.limits.max_xy_vel_mps
    _ = cfg.limits.max_z_vel_mps
    _ = cfg.limits.max_yaw_rate_dps
    _ = cfg.limits.position_tolerance_m
    _ = cfg.limits.altitude_tolerance_m

    # FailsafeConfig
    _ = cfg.failsafe.heartbeat_timeout_s

    # VisionConfig
    _ = cfg.vision.camera_index_or_path
    _ = cfg.vision.frame_width
    _ = cfg.vision.frame_height
    _ = cfg.vision.stream.host
    _ = cfg.vision.stream.fps
    _ = cfg.vision.stream.enabled

    # TelemetryConfig
    _ = cfg.telemetry.sample_rate_hz

    # LoggingConfig
    _ = cfg.logging.dir
    _ = cfg.logging.max_bytes
    _ = cfg.logging.backup_count

    print("[PASS] test_config_fields")


if __name__ == "__main__":
    test_load_default_config()
    test_config_fields()
    print("\nAll config tests passed.")
