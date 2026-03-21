from __future__ import annotations

import logging
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.config_loader import AppConfig
from src.fc_link import FlightControllerLink
from src.telemetry import TelemetryHub

if TYPE_CHECKING:
    from src.safety import SafetyManager


class CancelledError(RuntimeError):
    """任务被取消时抛出。"""


class CancelToken:
    """跨线程取消令牌。"""

    def __init__(self) -> None:
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._reason: str | None = None

    def cancel(self, reason: str) -> None:
        with self._lock:
            if self._event.is_set():
                return
            self._reason = reason
            self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason

    def raise_if_cancelled(self) -> None:
        if self.is_cancelled():
            raise CancelledError(self.reason or "cancelled")


@dataclass
class MissionContext:
    cfg: AppConfig
    link: FlightControllerLink
    safety: "SafetyManager"
    telemetry: TelemetryHub
    logger: logging.Logger
    cancel: CancelToken
    vision_bus: object | None = None

    @property
    def session(self):
        return self.link.get_session()


class MissionTask(ABC):
    name: str = "base"

    @abstractmethod
    def setup(self, ctx: MissionContext) -> None:
        """任务初始化。此阶段应保持轻量。"""

    @abstractmethod
    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        """任务主逻辑。必须频繁检查 token.is_cancelled()。"""

    @abstractmethod
    def teardown(self, ctx: MissionContext) -> None:
        """任务收尾。"""


class StandbyMission(MissionTask):
    name = "none"

    def setup(self, ctx: MissionContext) -> None:
        ctx.logger.info("Standby mission setup")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        del token
        ctx.logger.info("Standby mode: no mission action")

    def teardown(self, ctx: MissionContext) -> None:
        ctx.logger.info("Standby mission teardown")


def build_mission(name: str, ctx: MissionContext) -> MissionTask:
    normalized = (name or "none").strip().lower()
    if normalized in {"none", "standby"}:
        return StandbyMission()
    if normalized == "takeoff_and_hover":
        from src.missions.takeoff_and_hover import TakeoffAndHoverMission

        return TakeoffAndHoverMission()
    if normalized == "waypoint_square":
        from src.missions.waypoint_square import WaypointSquareMission

        return WaypointSquareMission()
    if normalized == "vision_track":
        from src.missions.vision_track import VisionTrackMission

        return VisionTrackMission()
    raise ValueError(f"未知 mission: {name}")
