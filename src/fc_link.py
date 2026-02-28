"""
fc_link — 飞控连接、重连与心跳监控

职责：
    管理与 ArduCopter 飞控之间的 MAVLink 串口连接，
    提供连接/断开/重连/心跳监控能力。

硬件前提：
    树莓派 UART ↔ Matek H743 Slim V3 UART

设计原则：
    - 连接超时、重试间隔、最大重试次数均可配
    - 连接成功后执行 ``wait_ready`` 等待关键字段可用
    - 支持 dry-run 模式（使用 FakeVehicle 代替真实连接）
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from src.config_loader import AppConfig

logger = logging.getLogger(__name__)


class FlightControllerLink:
    """飞控连接管理器。

    Usage::

        link = FlightControllerLink(cfg)
        link.connect()
        vehicle = link.get_vehicle()
        ...
        link.disconnect()

    Args:
        cfg:     全局配置实例。
        dry_run: 若为 True，使用 FakeVehicle 而非真实串口连接。
    """

    def __init__(self, cfg: AppConfig, dry_run: bool = False) -> None:
        self._cfg = cfg
        self._dry_run = dry_run
        self._vehicle: Any = None          # dronekit.Vehicle | FakeVehicle
        self._connected: bool = False
        self._last_heartbeat_ts: float = 0.0

    # ── 公开方法 ──────────────────────────────

    def connect(self) -> None:
        """连接飞控。

        在 dry-run 模式下创建 FakeVehicle；否则通过 DroneKit 连接
        串口，失败时按配置重试。

        Raises:
            ConnectionError: 所有重试均失败后抛出。
        """
        if self._connected:
            logger.warning("已连接，跳过重复连接")
            return

        if self._dry_run:
            self._connect_fake()
        else:
            self._connect_real()

    def disconnect(self) -> None:
        """安全断开飞控连接。"""
        if self._vehicle is not None:
            try:
                if not self._dry_run:
                    self._vehicle.close()
                logger.info("飞控连接已断开")
            except Exception as exc:
                logger.warning("断开连接时异常: %s", exc)
            finally:
                self._vehicle = None
                self._connected = False

    def is_connected(self) -> bool:
        """返回当前是否已连接飞控。"""
        return self._connected

    def get_vehicle(self) -> Any:
        """返回 DroneKit Vehicle 实例（或 FakeVehicle）。

        Returns:
            dronekit.Vehicle 或 FakeVehicle 实例。

        Raises:
            RuntimeError: 未连接时调用抛出。
        """
        self.ensure_connected_or_raise()
        return self._vehicle

    def last_heartbeat_age_s(self) -> float:
        """返回距上次心跳的秒数。

        Returns:
            float: 距最近一次心跳的时间（秒）。
                   如果从未收到心跳则返回 ``float('inf')``。
        """
        if self._dry_run:
            return 0.0  # FakeVehicle 永远在线
        if self._vehicle is None:
            return float("inf")
        try:
            return self._vehicle.last_heartbeat
        except Exception:
            return float("inf")

    def ensure_connected_or_raise(self) -> None:
        """确保当前已连接，否则抛出 RuntimeError。

        Raises:
            RuntimeError: 飞控未连接。
        """
        if not self._connected or self._vehicle is None:
            raise RuntimeError("飞控未连接，请先调用 connect()")

    # ── 内部方法 ──────────────────────────────

    def _connect_real(self) -> None:
        """通过 DroneKit 连接真实飞控串口。"""
        from dronekit import connect as dk_connect

        port = self._cfg.mavlink.port
        baud = self._cfg.mavlink.baud
        timeout = self._cfg.mavlink.connect_timeout_s
        retries = self._cfg.mavlink.max_retries
        interval = self._cfg.mavlink.retry_interval_s

        for attempt in range(1, retries + 1):
            logger.info("正在连接飞控  port=%s  baud=%d  (%d/%d)",
                        port, baud, attempt, retries)
            try:
                self._vehicle = dk_connect(
                    port,
                    baud=baud,
                    wait_ready=True,
                    heartbeat_timeout=timeout,
                )
                self._connected = True
                self._last_heartbeat_ts = time.time()
                logger.info("飞控连接成功  mode=%s  armed=%s",
                            self._vehicle.mode.name, self._vehicle.armed)
                return
            except Exception as exc:
                logger.error("连接失败 (%d/%d): %s", attempt, retries, exc)
                if attempt < retries:
                    logger.info("等待 %ds 后重试…", interval)
                    time.sleep(interval)

        raise ConnectionError(
            f"无法连接飞控（已重试 {retries} 次）: {port}"
        )

    def _connect_fake(self) -> None:
        """创建 FakeVehicle 用于 dry-run 测试。"""
        from tests.fake_vehicle import FakeVehicle

        logger.info("[DRY-RUN] 使用 FakeVehicle 模拟飞控连接")
        self._vehicle = FakeVehicle()
        self._connected = True
        self._last_heartbeat_ts = time.time()
        logger.info("[DRY-RUN] FakeVehicle 就绪  mode=%s  armed=%s",
                    self._vehicle.mode.name, self._vehicle.armed)
