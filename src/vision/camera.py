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

    当摄像头不可用时，不会抛出异常，而是生成黑色占位帧，
    确保推流端口仍可正常启动并返回画面。

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
        self._camera_ok: bool = False
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ── 生命周期 ──────────────────────────────

    def start(self) -> None:
        """启动摄像头采集线程。

        摄像头打不开时不会抛出异常，而是回退到占位黑帧模式，
        确保推流仍可正常启动。
        """
        if self._running:
            return

        source = self._source
        logger.info("打开摄像头  source=%s  分辨率=%dx%d",
                     source, self._width, self._height)

        self._cap = cv2.VideoCapture(source if isinstance(source, str) else int(source))
        if not self._cap.isOpened():
            logger.warning("无法打开摄像头: %s — 回退到占位黑帧模式", source)
            self._camera_ok = False
            if self._cap is not None:
                self._cap.release()
                self._cap = None
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._camera_ok = True

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop,
            name="CameraSource",
            daemon=True,
        )
        self._thread.start()
        if self._camera_ok:
            logger.info("摄像头采集已启动")
        else:
            logger.info("摄像头采集已启动（占位黑帧模式）")

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

    @property
    def camera_ok(self) -> bool:
        """摄像头是否真正打开（非占位模式）。"""
        return self._camera_ok

    # ── 内部采集循环 ──────────────────────────

    def _capture_loop(self) -> None:
        """后台采集主循环。

        摄像头正常时从设备读取帧；摄像头不可用时生成黑色占位帧。
        """
        while self._running:
            if not self._camera_ok:
                # 生成黑色占位帧
                placeholder = np.zeros(
                    (self._height, self._width, 3), dtype=np.uint8,
                )
                with self._lock:
                    self._frame = placeholder
                time.sleep(0.1)  # 占位模式 ~10 fps
                continue

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
