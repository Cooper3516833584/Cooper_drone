"""Fake MAVLink objects for tests."""

from __future__ import annotations

import queue
from dataclasses import dataclass
from typing import Any


@dataclass
class FakeMavlinkMessage:
    """Simple fake MAVLink message with dynamic fields."""

    msg_type: str

    def __init__(self, msg_type: str, **fields: Any) -> None:
        """Create a fake MAVLink message."""
        self.msg_type = msg_type
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        """Return the fake MAVLink message type."""
        return self.msg_type


class FakeMavlinkConnection:
    """Queue-backed fake MAVLink connection."""

    def __init__(self, messages: list[FakeMavlinkMessage] | None = None) -> None:
        """Create a fake connection with optional queued messages."""
        self.messages: queue.Queue[FakeMavlinkMessage] = queue.Queue()
        self.mav = FakeMavlinkSender(self)
        self.closed = False
        self.target_system = 1
        self.target_component = 1
        self.wait_heartbeat_calls = 0
        self.recv_match_calls = 0
        for msg in messages or []:
            self.messages.put(msg)

    def add_message(self, msg: FakeMavlinkMessage) -> None:
        """Append a fake message to the receive queue."""
        self.messages.put(msg)

    def wait_heartbeat(self, timeout: float | None = None) -> FakeMavlinkMessage | None:
        """Return the next heartbeat from the queue."""
        self.wait_heartbeat_calls += 1
        try:
            while True:
                msg = self.messages.get(timeout=timeout or 0.01)
                if msg.get_type() == "HEARTBEAT":
                    return msg
        except queue.Empty:
            return None

    def recv_match(self, blocking: bool = False, timeout: float | None = None) -> FakeMavlinkMessage | None:
        """Return the next queued message."""
        self.recv_match_calls += 1
        wait_time = timeout if blocking else 0
        try:
            return self.messages.get(timeout=wait_time or 0.0)
        except queue.Empty:
            return None

    def close(self) -> None:
        """Mark the fake connection as closed."""
        self.closed = True


class FakeMavlinkSender:
    """Fake MAVLink sender attached to a fake connection."""

    def __init__(self, connection: FakeMavlinkConnection) -> None:
        """Create a fake MAVLink sender."""
        self._connection = connection
        self.command_long_calls: list[dict[str, Any]] = []
        self.body_velocity_calls: list[dict[str, Any]] = []
        self.ack_results: queue.Queue[int] = queue.Queue()
        self.auto_ack = True

    def add_ack_result(self, result: int) -> None:
        """Queue an ACK result for the next command."""
        self.ack_results.put(result)

    def command_long_send(
        self,
        target_system: int,
        target_component: int,
        command: int,
        confirmation: int,
        param1: float,
        param2: float,
        param3: float,
        param4: float,
        param5: float,
        param6: float,
        param7: float,
    ) -> None:
        """Capture a fake COMMAND_LONG send and optionally enqueue an ACK."""
        params = [param1, param2, param3, param4, param5, param6, param7]
        self.command_long_calls.append(
            {
                "target_system": target_system,
                "target_component": target_component,
                "command": command,
                "confirmation": confirmation,
                "params": params,
            }
        )
        if self.auto_ack:
            try:
                result = self.ack_results.get_nowait()
            except queue.Empty:
                result = 0
            self._connection.add_message(FakeMavlinkMessage("COMMAND_ACK", command=command, result=result))

    def set_position_target_local_ned_send(
        self,
        time_boot_ms: int,
        target_system: int,
        target_component: int,
        coordinate_frame: int,
        type_mask: int,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        afx: float,
        afy: float,
        afz: float,
        yaw: float,
        yaw_rate: float,
    ) -> None:
        """Capture a fake body velocity setpoint."""
        self.body_velocity_calls.append(
            {
                "time_boot_ms": time_boot_ms,
                "target_system": target_system,
                "target_component": target_component,
                "coordinate_frame": coordinate_frame,
                "type_mask": type_mask,
                "vx_mps": vx,
                "vy_mps": vy,
                "vz_mps": vz,
                "yaw_rate_rad_s": yaw_rate,
            }
        )
