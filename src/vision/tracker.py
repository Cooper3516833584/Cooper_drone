"""
tracker — 目标检测/跟踪

职责：
    后台线程从 CameraSource 读取最新帧，执行目标检测或跟踪，
    将结果输出为 VisionTarget（偏差 + 置信度 + 时间戳）。
    **只输出偏差，不直接控飞机。**

设计原则：
    - 与飞控完全解耦——Mission 层读取 VisionTarget 后再调用 control 原语
    - VisionTarget 必须带 timestamp，过期判定 is_stale
    - 控制循环不允许被视觉阻塞
    - 当前为占位实现，未来可替换为 YOLO / 颜色追踪 / ArUco 等算法
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING

from src.types import VisionTarget

if TYPE_CHECKING:
    from src.vision.camera import CameraSource

logger = logging.getLogger(__name__)


class Tracker:
    """目标跟踪器——后台线程持续检测，输出 VisionTarget。

    Usage::

        tracker = Tracker(camera)
        tracker.start()
        target = tracker.latest_target()
        if target.detected and not target.is_stale():
            # 使用 dx, dy 计算速度指令
            ...
        tracker.stop()

    Args:
        camera: CameraSource 实例（帧来源）。
    """

    def __init__(self, camera: CameraSource) -> None:
        self._camera = camera
        self._target = VisionTarget()
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    # ── 生命周期 ──────────────────────────────

    def start(self) -> None:
        """启动跟踪后台线程。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._track_loop,
            name="Tracker",
            daemon=True,
        )
        self._thread.start()
        logger.info("目标跟踪器已启动")

    def stop(self) -> None:
        """停止跟踪线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        logger.info("目标跟踪器已停止")

    # ── 查询接口 ──────────────────────────────

    def latest_target(self) -> VisionTarget:
        """获取最新目标检测结果（线程安全，返回拷贝）。

        Returns:
            VisionTarget 的拷贝实例。调用方应检查
            ``detected`` 和 ``is_stale()``。
        """
        from dataclasses import replace
        with self._lock:
            return replace(self._target)

    # ── 内部跟踪循环 ──────────────────────────

    def _track_loop(self) -> None:
        """后台跟踪主循环。

        当前为占位实现：
            - 从 CameraSource 获取最新帧
            - TODO: 替换为真实检测算法（颜色/YOLO/ArUco 等）
            - 输出 VisionTarget
        """
        while self._running:
            try:
                frame = self._camera.latest_frame()
                if frame is not None:
                    target = self._detect(frame)
                    with self._lock:
                        self._target = target
            except Exception as exc:
                logger.debug("跟踪异常: %s", exc)

            time.sleep(0.05)  # ~20 Hz 检测频率

    @staticmethod
    def _detect(frame) -> VisionTarget:
        """占位检测函数。

        TODO: 实现真实目标检测算法。
        可选方案：
            1. 颜色阈值 + 轮廓检测（最简单）
            2. ArUco 标记检测
            3. YOLO/TFLite 目标检测
            4. 模板匹配

        当前行为：
            返回未检测到目标的 VisionTarget。

        Args:
            frame: BGR 图像帧（numpy ndarray）。

        Returns:
            VisionTarget 实例。
        """
        return VisionTarget(
            timestamp=time.time(),
            dx=0.0,
            dy=0.0,
            confidence=0.0,
            detected=False,
        )
