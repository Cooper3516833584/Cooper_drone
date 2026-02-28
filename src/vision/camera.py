"""
camera — 摄像头采集（帧缓存）

职责：
    后台线程持续从摄像头（或视频文件）读取帧，
    存入帧缓存，供 Tracker 和 Flask 推流读取。

设计原则：
    - 控制循环不允许被视觉阻塞
    - 帧缓存仅保留最新一帧（latest_frame）
    - 线程安全
    - 支持 camera_index（整数）或视频文件路径（字符串）
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    from src.config_loader import AppConfig

logger = logging.getLogger(__name__)


class CameraSource:
    """摄像头采集源——后台线程持续采集，帧缓存。

    Usage::

        cam = CameraSource(cfg)
        cam.start()
        frame = cam.latest_frame()   # numpy ndarray or None
        cam.stop()

    Args:
        cfg: 全局配置实例。
    """

    def __init__(self, cfg: AppConfig) -> None:
        self._cfg = cfg
        self._source = cfg.vision.camera_index_or_path
        self._width = cfg.vision.frame_width
        self._height = cfg.vision.frame_height

        self._cap: cv2.VideoCapture | None = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ── 生命周期 ──────────────────────────────

    def start(self) -> None:
        """启动摄像头采集线程。

        Raises:
            RuntimeError: 摄像头无法打开。
        """
        if self._running:
            return

        source = self._source
        logger.info("打开摄像头  source=%s  分辨率=%dx%d",
                     source, self._width, self._height)

        self._cap = cv2.VideoCapture(source if isinstance(source, str) else int(source))
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开摄像头: {source}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="CameraSource",
            daemon=True,
        )
        self._thread.start()
        logger.info("摄像头采集已启动")

    def stop(self) -> None:
        """停止摄像头采集并释放资源。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("摄像头采集已停止")

    # ── 查询接口 ──────────────────────────────

    def latest_frame(self) -> np.ndarray | None:
        """获取最新一帧（线程安全）。

        Returns:
            numpy ndarray (BGR)，尚无帧时返回 None。
        """
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def is_running(self) -> bool:
        """摄像头是否正在采集。"""
        return self._running

    # ── 内部采集循环 ──────────────────────────

    def _capture_loop(self) -> None:
        """后台采集主循环。"""
        while self._running:
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
            else:
                logger.debug("摄像头读帧失败")
                time.sleep(0.05)
