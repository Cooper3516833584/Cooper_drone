from __future__ import annotations

import copy
import logging
import threading
import time
from typing import Callable

from src.mav_state import VehicleStateCache
from src.types import TelemetryView


class TelemetryHub:
    """从状态缓存提取并发布只读遥测快照。"""

    def __init__(self, state_cache: VehicleStateCache, sample_hz: float, logger: logging.Logger | None = None) -> None:
        self._state_cache = state_cache
        self._sample_period_s = 1.0 / max(0.1, sample_hz)
        self._log = logger or logging.getLogger(__name__)

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._latest: TelemetryView | None = None
        self._subscribers: list[Callable[[TelemetryView], None]] = []

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, name="telemetry-hub", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

    def subscribe(self, callback: Callable[[TelemetryView], None]) -> None:
        with self._lock:
            self._subscribers.append(callback)

    def latest(self) -> TelemetryView | None:
        with self._lock:
            return copy.deepcopy(self._latest)

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            view = self._build_view()
            callbacks: list[Callable[[TelemetryView], None]]
            with self._lock:
                self._latest = view
                callbacks = list(self._subscribers)

            for callback in callbacks:
                try:
                    callback(view)
                except Exception as exc:
                    self._log.warning("telemetry callback error: %s", exc)

            time.sleep(self._sample_period_s)

    def _build_view(self) -> TelemetryView:
        now = time.monotonic()
        state = self._state_cache.snapshot()
        heartbeat_age = None
        if state.last_heartbeat_monotonic is not None:
            heartbeat_age = max(0.0, now - state.last_heartbeat_monotonic)

        return TelemetryView(
            timestamp_monotonic=now,
            mode=state.mode,
            armed=state.armed,
            lat_deg=state.position.lat_deg,
            lon_deg=state.position.lon_deg,
            alt_m=state.position.alt_m,
            relative_alt_m=state.position.relative_alt_m,
            vx_mps=state.velocity.vx_mps,
            vy_mps=state.velocity.vy_mps,
            vz_mps=state.velocity.vz_mps,
            battery_voltage_v=state.battery.voltage_v,
            battery_remaining_pct=state.battery.remaining_pct,
            gps_fix_type=state.gps_fix_type,
            satellites_visible=state.satellites_visible,
            rc_channels=dict(state.rc.channels),
            heartbeat_age_s=heartbeat_age,
            ekf_healthy=state.ekf_healthy,
            latest_status_text=state.latest_status_text,
        )

