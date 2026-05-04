"""Low-level MAVLink connection wrapper."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

from src.config_loader import MavlinkConfig
from src.logging_runtime import RuntimeLogger
from src.mavlink_messages import message_type
from src.state import VehicleState, VehicleStateCache


class MavlinkConnectionError(RuntimeError):
    """Raised when a MAVLink connection operation fails."""


class MavlinkConnection:
    """Owns the low-level MAVLink connection and receiver loop."""

    def __init__(
        self,
        cfg: MavlinkConfig,
        logger: RuntimeLogger,
        connection_factory: Callable[..., Any] | None = None,
    ) -> None:
        """Create a MAVLink connection wrapper."""
        self._cfg = cfg
        self._logger = logger
        self._connection_factory = connection_factory
        self._connection: Any | None = None
        self._state_cache = VehicleStateCache()
        self._receiver_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._connection_lock = threading.Lock()
        self._handlers_lock = threading.Lock()
        self._message_handlers: dict[str, list[Callable[[Any], None]]] = {}

    def connect(self) -> None:
        """Open the MAVLink connection unless this is a dry-run profile."""
        if self._is_dry_run:
            self._logger.event("mavlink_connection_skipped", reason="dry_run")
            return

        with self._connection_lock:
            if self._connection is not None:
                return

            factory = self._connection_factory or _default_mavlink_connection_factory()
            self._connection = factory(
                self._cfg.connection_string,
                baud=self._cfg.baud,
                source_system=self._cfg.source_system,
                source_component=self._cfg.source_component,
            )
            self._logger.event("mavlink_connected", connection_string=self._cfg.connection_string)

    def close(self) -> None:
        """Stop receiving and close the MAVLink connection."""
        self.stop_receiver()
        with self._connection_lock:
            connection = self._connection
            self._connection = None

        if connection is not None and hasattr(connection, "close"):
            try:
                connection.close()
            except Exception:
                self._logger.app_logger.exception("Failed to close MAVLink connection")

    def wait_heartbeat(self, timeout_s: float | None = None) -> VehicleState:
        """Wait for a heartbeat and return the updated vehicle state."""
        if self._is_dry_run:
            return self.state_snapshot()

        connection = self._require_connection()
        timeout = self._cfg.heartbeat_timeout_s if timeout_s is None else timeout_s
        msg = connection.wait_heartbeat(timeout=timeout)
        if msg is None:
            raise MavlinkConnectionError("Timed out waiting for MAVLink heartbeat")

        self._state_cache.update_from_message(msg)
        self._logger.event("mavlink_heartbeat_received")
        return self.state_snapshot()

    def start_receiver(self) -> None:
        """Start the background MAVLink receive loop."""
        if self._is_dry_run:
            self._logger.event("mavlink_receiver_skipped", reason="dry_run")
            return

        self._require_connection()
        if self._receiver_thread is not None and self._receiver_thread.is_alive():
            return

        self._stop_event.clear()
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop,
            name="mavlink-receiver",
            daemon=True,
        )
        self._receiver_thread.start()

    def stop_receiver(self) -> None:
        """Stop the background MAVLink receive loop."""
        self._stop_event.set()
        thread = self._receiver_thread
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self._logger.app_logger.error("MAVLink receiver thread did not stop before timeout")
        self._receiver_thread = None

    def state_snapshot(self) -> VehicleState:
        """Return a copy of the latest vehicle state."""
        return self._state_cache.snapshot()

    def add_message_handler(self, msg_type: str, handler: Callable[[Any], None]) -> None:
        """Register a callback for a MAVLink message type."""
        with self._handlers_lock:
            self._message_handlers.setdefault(msg_type, []).append(handler)

    def send_command_long(self, command: int, params: list[float]) -> None:
        """Send a low-level MAVLink COMMAND_LONG message."""
        connection = self._require_connection()
        target_system = self._cfg.target_system or getattr(connection, "target_system", 1)
        target_component = self._cfg.target_component or getattr(connection, "target_component", 1)
        connection.mav.command_long_send(
            target_system,
            target_component,
            command,
            0,
            params[0],
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6],
        )

    def send_body_velocity_setpoint(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
    ) -> None:
        """Send one low-level body-frame velocity setpoint."""
        connection = self._require_connection()
        target_system = self._cfg.target_system or getattr(connection, "target_system", 1)
        target_component = self._cfg.target_component or getattr(connection, "target_component", 1)
        time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        mav_frame_body_ned = 8
        type_mask_velocity_yaw_rate_only = 1479
        yaw_rate_rad_s = math.radians(yaw_rate_dps)
        connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            mav_frame_body_ned,
            type_mask_velocity_yaw_rate_only,
            0.0,
            0.0,
            0.0,
            vx_mps,
            vy_mps,
            vz_mps,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate_rad_s,
        )

    @property
    def is_dry_run(self) -> bool:
        """Return whether this connection is configured for dry-run."""
        return self._is_dry_run

    @property
    def command_timeout_s(self) -> float:
        """Return the configured command timeout."""
        return self._cfg.command_timeout_s

    @property
    def command_retries(self) -> int:
        """Return the configured command retry count."""
        return self._cfg.command_retries

    @property
    def _is_dry_run(self) -> bool:
        return self._cfg.connection_string == "dry-run"

    def _require_connection(self) -> Any:
        if self._connection is None:
            raise MavlinkConnectionError("MAVLink connection is not open")
        return self._connection

    def _receiver_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                connection = self._require_connection()
                msg = connection.recv_match(blocking=True, timeout=0.1)
                if msg is not None:
                    self._state_cache.update_from_message(msg)
                    self._dispatch_message(msg)
            except MavlinkConnectionError:
                self._logger.app_logger.exception("MAVLink receiver has no active connection")
                time.sleep(0.1)
            except Exception:
                self._logger.app_logger.exception("MAVLink receiver failed to process a message")
                time.sleep(0.1)

    def _dispatch_message(self, msg: Any) -> None:
        msg_type = message_type(msg)
        if msg_type is None:
            return

        with self._handlers_lock:
            handlers = list(self._message_handlers.get(msg_type, []))

        for handler in handlers:
            try:
                handler(msg)
            except Exception:
                self._logger.app_logger.exception("MAVLink message handler failed for %s", msg_type)


def _default_mavlink_connection_factory() -> Callable[..., Any]:
    from pymavlink import mavutil

    return mavutil.mavlink_connection
