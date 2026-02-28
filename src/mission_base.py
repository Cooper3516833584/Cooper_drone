"""
mission_base — 任务基类、取消令牌与任务上下文

职责：
    定义所有飞行任务的统一生命周期：setup -> run -> teardown。
    提供 CancelToken 机制，让 SafetyManager、超时或外部信号可随时取消任务。
    提供 MissionContext，聚合所有子系统引用，供任务使用。

设计原则：
    - 所有任务统一：可取消、可超时、可安全收尾
    - teardown 无论成功/失败/取消都必须执行
    - 任务内部应频繁检查 ``token.is_cancelled()``
"""

from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from src.config_loader import AppConfig
    from src.fc_link import FlightControllerLink
    from src.safety import SafetyManager
    from src.telemetry import TelemetryHub

logger = logging.getLogger(__name__)


# ── CancelToken ──────────────────────────────────────────


class CancelToken:
    """任务取消令牌——线程安全。

    Usage::

        token = CancelToken()
        safety.register_cancel_token(token)

        # 在任务循环中检查
        while not token.is_cancelled():
            ...

        # 外部取消
        token.cancel("用户撤销接管")
    """

    def __init__(self) -> None:
        self._cancelled = False
        self._reason: str = ""
        self._lock = threading.Lock()

    def cancel(self, reason: str = "未指定原因") -> None:
        """触发取消。

        Args:
            reason: 取消原因描述。
        """
        with self._lock:
            if not self._cancelled:
                self._cancelled = True
                self._reason = reason
                logger.info("CancelToken 已触发: %s", reason)

    def is_cancelled(self) -> bool:
        """是否已取消。"""
        with self._lock:
            return self._cancelled

    def reason(self) -> str:
        """取消原因。"""
        with self._lock:
            return self._reason

    def reset(self) -> None:
        """重置令牌（慎用，仅用于复用场景）。"""
        with self._lock:
            self._cancelled = False
            self._reason = ""


# ── MissionContext ───────────────────────────────────────


class MissionContext:
    """任务执行上下文——聚合所有子系统引用。

    任务通过此对象访问配置、飞控、安全层、遥测等，
    避免在任务代码中直接 import 各子模块。

    Attributes:
        cfg:        全局配置实例
        link:       飞控连接管理器
        safety:     安全管理器
        telemetry:  遥测中心
        vision_bus: 视觉数据总线（VisionTarget 来源），可选
        logger:     任务专用 Logger
    """

    def __init__(
        self,
        cfg: AppConfig,
        link: FlightControllerLink,
        safety: SafetyManager,
        telemetry: TelemetryHub,
        vision_bus: Any = None,
        task_logger: logging.Logger | None = None,
    ) -> None:
        self.cfg = cfg
        self.link = link
        self.safety = safety
        self.telemetry = telemetry
        self.vision_bus = vision_bus
        self.logger = task_logger or logging.getLogger("mission")


# ── MissionTask (ABC) ───────────────────────────────────


class MissionTask(ABC):
    """飞行任务抽象基类。

    子类必须实现：
        - ``setup(ctx)``：任务初始化（如切换模式、检查前提）
        - ``run(ctx, token)``：任务主逻辑（必须频繁检查 token）
        - ``teardown(ctx)``：任务清理（始终执行，无论成功/失败/取消）

    生命周期::

        task = SomeTask()
        try:
            task.setup(ctx)
            task.run(ctx, token)
        except Exception:
            ...
        finally:
            task.teardown(ctx)

    Attributes:
        name:      任务名称（用于日志与显示）
        timeout_s: 任务超时（秒），0 表示无超时
    """

    def __init__(self, name: str, timeout_s: float = 0.0) -> None:
        self.name = name
        self.timeout_s = timeout_s

    @abstractmethod
    def setup(self, ctx: MissionContext) -> None:
        """任务初始化。

        典型操作：
            - 检查当前飞行状态是否满足前提
            - 切换到需要的飞行模式
            - 准备任务所需资源

        Args:
            ctx: 任务执行上下文。

        Raises:
            RuntimeError: 前提条件不满足。
        """
        ...

    @abstractmethod
    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        """任务主逻辑。

        **必须频繁检查 ``token.is_cancelled()``**，以便安全层
        能随时中断任务。

        Args:
            ctx:   任务执行上下文。
            token: 取消令牌。

        Raises:
            TimeoutError: 任务超时。
            RuntimeError: 任务执行失败。
        """
        ...

    @abstractmethod
    def teardown(self, ctx: MissionContext) -> None:
        """任务清理——始终执行。

        典型操作：
            - 停止运动输出（stop_motion）
            - 释放任务专有资源
            - 记录任务结果

        Args:
            ctx: 任务执行上下文。
        """
        ...

    def execute(self, ctx: MissionContext, token: CancelToken) -> None:
        """完整执行任务（setup -> run -> teardown）。

        自动处理超时、取消、异常，并确保 teardown 始终执行。

        Args:
            ctx:   任务执行上下文。
            token: 取消令牌。
        """
        task_logger = ctx.logger
        task_logger.info("== 任务开始: %s  (超时=%.0fs) ==", self.name, self.timeout_s)

        # 超时守护线程
        timeout_timer: threading.Timer | None = None
        if self.timeout_s > 0:
            def _on_timeout() -> None:
                token.cancel(f"任务超时 ({self.timeout_s}s)")
            timeout_timer = threading.Timer(self.timeout_s, _on_timeout)
            timeout_timer.daemon = True
            timeout_timer.start()

        try:
            # ── Setup ──
            task_logger.info("> setup()")
            self.setup(ctx)

            # 检查 setup 后是否被取消
            if token.is_cancelled():
                task_logger.warning("任务在 setup 后被取消: %s", token.reason())
                return

            # ── Run ──
            task_logger.info("> run()")
            self.run(ctx, token)

            if token.is_cancelled():
                task_logger.warning("任务被取消: %s", token.reason())
            else:
                task_logger.info("任务正常完成: %s", self.name)

        except Exception as exc:
            task_logger.error("任务异常: %s -- %s", self.name, exc, exc_info=True)
            if not token.is_cancelled():
                token.cancel(f"任务异常: {exc}")

        finally:
            # ── Teardown（始终执行）──
            if timeout_timer is not None:
                timeout_timer.cancel()
            try:
                task_logger.info("> teardown()")
                self.teardown(ctx)
            except Exception as exc:
                task_logger.error("teardown 异常: %s", exc, exc_info=True)

            task_logger.info("== 任务结束: %s ==", self.name)
