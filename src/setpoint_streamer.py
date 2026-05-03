"""Body velocity setpoint streamer."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from src.logging_runtime import RuntimeLogger
from src.movement import DroneMovement


@dataclass
class BodyVelocitySetpoint:
    """Latest body velocity setpoint with update timestamp."""

    vx_mps: float
    vy_mps: float
    vz_mps: float
    yaw_rate_dps: float
    ts: float


class BodyVelocityStreamer:
    """Thread-safe streamer that resends the latest body velocity setpoint."""

    def __init__(
        self,
        movement: DroneMovement,
        logger: RuntimeLogger,
        *,
        rate_hz: float,
        setpoint_ttl_s: float,
    ) -> None:
        """Create a body velocity streamer."""
        if rate_hz <= 0:
            raise ValueError("rate_hz must be greater than 0")
        if setpoint_ttl_s <= 0:
            raise ValueError("setpoint_ttl_s must be greater than 0")

        self._movement = movement
        self._logger = logger
        self._rate_hz = rate_hz
        self._setpoint_ttl_s = setpoint_ttl_s
        self._period_s = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: BodyVelocitySetpoint | None = None

    def start(self) -> None:
        """Start the streamer thread."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, name="body-velocity-streamer", daemon=False)
            self._thread.start()
        self._logger.event("body_velocity_streamer_started", rate_hz=self._rate_hz, ttl_s=self._setpoint_ttl_s)

    def update(self, vx_mps: float, vy_mps: float, vz_mps: float, yaw_rate_dps: float = 0.0) -> None:
        """Update the latest setpoint without blocking on send."""
        setpoint = BodyVelocitySetpoint(
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            vz_mps=vz_mps,
            yaw_rate_dps=yaw_rate_dps,
            ts=time.monotonic(),
        )
        with self._lock:
            self._latest = setpoint
        self._logger.event(
            "body_velocity_setpoint_updated",
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            vz_mps=vz_mps,
            yaw_rate_dps=yaw_rate_dps,
        )

    def stop(self) -> None:
        """Send zero velocity, stop the streamer, and join the thread."""
        self._stop_event.set()
        self._send_zero("body_velocity_streamer_stop_zero")
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self._logger.event("body_velocity_streamer_stop_timeout")
        with self._lock:
            self._thread = None
        self._logger.event("body_velocity_streamer_stopped")

    def is_running(self) -> bool:
        """Return whether the streamer thread is running."""
        thread = self._thread
        return thread is not None and thread.is_alive()

    def _run(self) -> None:
        next_send = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_send:
                self._send_latest_or_zero(now)
                next_send = now + self._period_s
            self._stop_event.wait(max(0.0, min(self._period_s, next_send - time.monotonic())))

    def _send_latest_or_zero(self, now: float) -> None:
        with self._lock:
            latest = self._latest

        if latest is None or now - latest.ts > self._setpoint_ttl_s:
            self._send_zero("body_velocity_streamer_ttl_zero")
            return

        self._send(
            latest.vx_mps,
            latest.vy_mps,
            latest.vz_mps,
            latest.yaw_rate_dps,
            event_name="body_velocity_streamer_sent",
        )

    def _send_zero(self, event_name: str) -> None:
        self._send(0.0, 0.0, 0.0, 0.0, event_name=event_name)

    def _send(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
        *,
        event_name: str,
    ) -> None:
        try:
            self._movement.send_body_velocity(vx_mps, vy_mps, vz_mps, yaw_rate_dps)
            self._logger.event(
                event_name,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                vz_mps=vz_mps,
                yaw_rate_dps=yaw_rate_dps,
            )
        except Exception as exc:
            self._logger.event(
                "body_velocity_streamer_send_failed",
                error=repr(exc),
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                vz_mps=vz_mps,
                yaw_rate_dps=yaw_rate_dps,
            )
