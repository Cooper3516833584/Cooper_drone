"""
telemetry — 遥测采样与发布

职责：
    统一读取飞控状态（位置/速度/电池/GPS/EKF/RC 等），
    定时采样生成 VehicleSnapshot，供日志、任务、网页使用。
    避免各模块各自读取造成竞争或重复开销。

设计原则：
    - 后台线程按可配频率采样（如 5 Hz）
    - 线程安全：latest() 可被任何线程调用
    - 支持可选的回调订阅机制
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING, Any, Callable

from src.types import VehicleSnapshot

if TYPE_CHECKING:
    from src.config_loader import AppConfig
    from src.fc_link import FlightControllerLink

logger = logging.getLogger(__name__)


class TelemetryHub:
    """遥测采样中心——后台线程定时采集飞控状态。

    Usage::

        hub = TelemetryHub(cfg, link)
        hub.start()
        snap = hub.latest()
        print(snap.mode, snap.alt_rel_m)
        hub.stop()

    Args:
        cfg:  全局配置实例。
        link: 飞控连接管理器。
    """

    def __init__(self, cfg: AppConfig, link: FlightControllerLink) -> None:
        self._cfg = cfg
        self._link = link

        self._interval_s = 1.0 / max(cfg.telemetry.sample_rate_hz, 1)
        self._snapshot = VehicleSnapshot()
        self._lock = threading.Lock()
        self._subscribers: list[Callable[[VehicleSnapshot], None]] = []
        self._running = False
        self._thread: threading.Thread | None = None

    # ── 生命周期 ──────────────────────────────

    def start(self) -> None:
        """启动遥测采样后台线程。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._sample_loop,
            name="TelemetryHub",
            daemon=True,
        )
        self._thread.start()
        logger.info("遥测采样已启动  频率=%.1f Hz", 1.0 / self._interval_s)

    def stop(self) -> None:
        """停止遥测采样线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("遥测采样已停止")

    # ── 查询接口 ──────────────────────────────

    def latest(self) -> VehicleSnapshot:
        """获取最新遥测快照（线程安全）。

        Returns:
            最近一次采样的 VehicleSnapshot 副本。
        """
        with self._lock:
            return self._snapshot

    def subscribe(self, callback: Callable[[VehicleSnapshot], None]) -> None:
        """注册遥测回调。

        每次采样完成后会调用所有注册回调。

        Args:
            callback: 接收 VehicleSnapshot 的回调函数。
        """
        self._subscribers.append(callback)
        logger.debug("新增遥测订阅者，当前共 %d 个", len(self._subscribers))

    # ── 内部采样循环 ──────────────────────────

    def _sample_loop(self) -> None:
        """后台采样主循环。"""
        while self._running:
            try:
                if self._link.is_connected():
                    self._do_sample()
            except Exception as exc:
                logger.debug("遥测采样异常: %s", exc)
            time.sleep(self._interval_s)

    def _do_sample(self) -> None:
        """执行一次遥测采样。"""
        vehicle = self._link.get_vehicle()
        snap = self._build_snapshot(vehicle)

        with self._lock:
            self._snapshot = snap

        # 通知订阅者
        for cb in self._subscribers:
            try:
                cb(snap)
            except Exception as exc:
                logger.debug("遥测回调异常: %s", exc)

    @staticmethod
    def _build_snapshot(vehicle: Any) -> VehicleSnapshot:
        """从 DroneKit Vehicle 构建 VehicleSnapshot。

        Args:
            vehicle: DroneKit Vehicle 实例或 FakeVehicle。

        Returns:
            填充了当前状态的 VehicleSnapshot。
        """
        snap = VehicleSnapshot(timestamp=time.time())

        try:
            snap.mode = vehicle.mode.name
        except Exception:
            snap.mode = "UNKNOWN"

        try:
            snap.armed = bool(vehicle.armed)
        except Exception:
            pass

        # 位置
        try:
            loc = vehicle.location.global_relative_frame
            snap.lat = loc.lat or 0.0
            snap.lon = loc.lon or 0.0
            snap.alt_rel_m = loc.alt or 0.0
        except Exception:
            pass

        try:
            loc_msl = vehicle.location.global_frame
            snap.alt_msl_m = loc_msl.alt or 0.0
        except Exception:
            pass

        # 速度
        try:
            vel = vehicle.velocity
            if vel and len(vel) >= 3:
                snap.vx, snap.vy, snap.vz = vel[0], vel[1], vel[2]
        except Exception:
            pass

        # 航向
        try:
            snap.heading_deg = float(vehicle.heading or 0)
        except Exception:
            pass

        # 电池
        try:
            batt = vehicle.battery
            if batt:
                snap.battery_v = float(batt.voltage or 0)
                snap.battery_pct = float(batt.level) if batt.level is not None else -1.0
        except Exception:
            pass

        # GPS
        try:
            gps = vehicle.gps_0
            if gps:
                snap.gps_fix = int(gps.fix_type or 0)
                snap.gps_num_sat = int(gps.satellites_visible or 0)
        except Exception:
            pass

        # EKF
        try:
            snap.ekf_ok = bool(vehicle.ekf_ok)
        except Exception:
            pass

        # RC 通道
        try:
            channels = vehicle.channels
            if channels:
                snap.rc_channels = {int(k): int(v) for k, v in channels.items()}
        except Exception:
            pass

        return snap
