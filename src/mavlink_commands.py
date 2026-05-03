"""Low-level MAVLink command service."""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Sequence

from src.logging_runtime import RuntimeLogger
from src.mavlink_connection import MavlinkConnection


MAV_RESULT_ACCEPTED = 0
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
    9: "COMMAND_UNSUPPORTED_MAV_FRAME",
}


class MavlinkCommandError(RuntimeError):
    """Raised when a MAVLink command fails."""


class MavlinkCommandTimeout(TimeoutError):
    """Raised when a MAVLink command ACK is not received before timeout."""


class MavlinkCommandRejected(MavlinkCommandError):
    """Raised when a MAVLink command ACK reports a rejected result."""


@dataclass
class CommandAck:
    """Parsed COMMAND_ACK result."""

    command: int
    result: int
    result_name: str
    progress: int | None = None
    raw: Any | None = None


class MavlinkCommandService:
    """Low-level service for MAVLink command messages."""

    def __init__(self, connection: MavlinkConnection, logger: RuntimeLogger) -> None:
        """Create a command service bound to a MAVLink connection."""
        self._connection = connection
        self._logger = logger
        self._pending_lock = threading.Lock()
        self._pending_acks: dict[int, deque[queue.Queue[CommandAck]]] = {}
        self._connection.add_message_handler("COMMAND_ACK", self._handle_command_ack)

    def command_long(
        self,
        command: int,
        params: Sequence[float] = (),
        *,
        timeout_s: float | None = None,
        retries: int | None = None,
        name: str | None = None,
    ) -> CommandAck:
        """Send COMMAND_LONG and wait for the matching COMMAND_ACK."""
        command_name = name or f"COMMAND_{command}"
        padded_params = _pad_params(params)
        timeout = self._connection.command_timeout_s if timeout_s is None else timeout_s
        retry_count = self._connection.command_retries if retries is None else retries
        started = time.monotonic()

        if self._connection.is_dry_run:
            ack = CommandAck(command=command, result=MAV_RESULT_ACCEPTED, result_name="ACCEPTED")
            self._log_command(command_name, command, padded_params, ack, "dry_run", started, 1)
            return ack

        self._connection.start_receiver()
        ack_queue: queue.Queue[CommandAck] = queue.Queue(maxsize=1)
        self._register_waiter(command, ack_queue)
        attempts = retry_count + 1
        try:
            for attempt in range(1, attempts + 1):
                self._connection.send_command_long(command, padded_params)
                try:
                    ack = ack_queue.get(timeout=timeout)
                except queue.Empty:
                    if attempt == attempts:
                        duration_ms = _duration_ms(started)
                        self._log_timeout(command_name, command, padded_params, duration_ms, attempt)
                        raise MavlinkCommandTimeout(
                            f"Timed out waiting for COMMAND_ACK command={command} after {attempt} attempts"
                        )
                    continue

                if ack.result != MAV_RESULT_ACCEPTED:
                    self._log_command(command_name, command, padded_params, ack, "rejected", started, attempt)
                    raise MavlinkCommandRejected(
                        f"COMMAND_ACK rejected command={command} result={ack.result_name}"
                    )

                self._log_command(command_name, command, padded_params, ack, "accepted", started, attempt)
                return ack
        finally:
            self._remove_waiter(command, ack_queue)

        raise MavlinkCommandTimeout(f"Timed out waiting for COMMAND_ACK command={command}")

    def send_body_velocity_setpoint(
        self,
        vx_mps: float,
        vy_mps: float,
        vz_mps: float,
        yaw_rate_dps: float,
        *,
        name: str = "body_velocity_setpoint",
    ) -> None:
        """Send one body-frame velocity setpoint and log it."""
        started = time.monotonic()
        if self._connection.is_dry_run:
            self._logger.command(
                name,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                vz_mps=vz_mps,
                yaw_rate_dps=yaw_rate_dps,
                status="dry_run",
                duration_ms=_duration_ms(started),
            )
            return

        self._connection.send_body_velocity_setpoint(vx_mps, vy_mps, vz_mps, yaw_rate_dps)
        self._logger.command(
            name,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            vz_mps=vz_mps,
            yaw_rate_dps=yaw_rate_dps,
            status="sent",
            duration_ms=_duration_ms(started),
        )

    def _handle_command_ack(self, msg: Any) -> None:
        command = int(getattr(msg, "command", -1))
        ack = CommandAck(
            command=command,
            result=int(getattr(msg, "result", -1)),
            result_name=_result_name(int(getattr(msg, "result", -1))),
            progress=getattr(msg, "progress", None),
            raw=msg,
        )

        with self._pending_lock:
            waiters = self._pending_acks.get(command)
            if not waiters:
                self._logger.event("unknown_command_ack", command=command, result=ack.result_name)
                return
            ack_queue = waiters.popleft()
            if not waiters:
                self._pending_acks.pop(command, None)

        try:
            ack_queue.put_nowait(ack)
        except queue.Full:
            self._logger.app_logger.error("COMMAND_ACK queue is full for command=%s", command)

    def _register_waiter(self, command: int, ack_queue: queue.Queue[CommandAck]) -> None:
        with self._pending_lock:
            self._pending_acks.setdefault(command, deque()).append(ack_queue)

    def _remove_waiter(self, command: int, ack_queue: queue.Queue[CommandAck]) -> None:
        with self._pending_lock:
            waiters = self._pending_acks.get(command)
            if not waiters:
                return
            try:
                waiters.remove(ack_queue)
            except ValueError:
                pass
            if not waiters:
                self._pending_acks.pop(command, None)

    def _log_command(
        self,
        name: str,
        command: int,
        params: list[float],
        ack: CommandAck,
        status: str,
        started: float,
        attempts: int,
    ) -> None:
        self._logger.command(
            name,
            command=command,
            params=params,
            result=ack.result,
            result_name=ack.result_name,
            status=status,
            duration_ms=_duration_ms(started),
            attempts=attempts,
        )

    def _log_timeout(
        self,
        name: str,
        command: int,
        params: list[float],
        duration_ms: int,
        attempts: int,
    ) -> None:
        self._logger.command(
            name,
            command=command,
            params=params,
            result=None,
            result_name="TIMEOUT",
            status="timeout",
            duration_ms=duration_ms,
            attempts=attempts,
        )


def _pad_params(params: Sequence[float]) -> list[float]:
    if len(params) > 7:
        raise MavlinkCommandError("COMMAND_LONG supports at most 7 parameters")
    padded = [float(value) for value in params]
    padded.extend([0.0] * (7 - len(padded)))
    return padded


def _result_name(result: int) -> str:
    return MAV_RESULT_NAMES.get(result, f"UNKNOWN_{result}")


def _duration_ms(started: float) -> int:
    return int((time.monotonic() - started) * 1000)
