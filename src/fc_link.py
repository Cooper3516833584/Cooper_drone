from __future__ import annotations

import logging

from src.config_loader import MavlinkConfig
from src.mav_session import FakeMavSession, MavSession, SessionLike
from src.mav_state import VehicleStateCache


class FlightControllerLink:
    """飞控链路包装器：统一管理真实/模拟 session。"""

    def __init__(
        self,
        mav_cfg: MavlinkConfig,
        state_cache: VehicleStateCache,
        *,
        dry_run: bool = False,
        logger: logging.Logger | None = None,
    ) -> None:
        self._mav_cfg = mav_cfg
        self._state_cache = state_cache
        self._dry_run = dry_run
        self._log = logger or logging.getLogger(__name__)
        self._session: SessionLike | None = None

    @property
    def state_cache(self) -> VehicleStateCache:
        return self._state_cache

    def connect(self) -> None:
        if self._session is not None and self._session.is_connected():
            return
        self._session = (
            FakeMavSession(self._mav_cfg, self._state_cache, logger=self._log)
            if self._dry_run
            else MavSession(self._mav_cfg, self._state_cache, logger=self._log)
        )
        self._session.connect()

    def close(self) -> None:
        if self._session is not None:
            self._session.close()

    def is_connected(self) -> bool:
        return self._session is not None and self._session.is_connected()

    def last_heartbeat_age_s(self) -> float:
        if self._session is None:
            return float("inf")
        return self._session.last_heartbeat_age_s()

    def get_session(self) -> SessionLike:
        if self._session is None:
            raise RuntimeError("session 尚未初始化，请先调用 connect()")
        return self._session

