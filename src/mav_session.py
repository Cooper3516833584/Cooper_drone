from __future__ import annotations

import logging
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable, Protocol

from src.config_loader import MavlinkConfig
from src.mav_state import VehicleState, VehicleStateCache

try:
    from pymavlink import mavutil
except Exception:  # pragma: no cover
    mavutil = None


@dataclass(frozen=True)
class CommandAck:
    command: int
    result: int


class SessionLike(Protocol):
    state_cache: VehicleStateCache

    def connect(self) -> None: ...

    def close(self) -> None: ...

    def is_connected(self) -> bool: ...

    def last_heartbeat_age_s(self) -> float: ...

    def request_message_intervals(self) -> None: ...

    def set_mode(self, mode: str, *, timeout_s: float = 5.0) -> None: ...

    def command_long(
        self,
        command: int,
        params: Iterable[float],
        *,
        timeout_s: float | None = None,
        wait_ack: bool = True,
    ) -> CommandAck: ...

    def wait_command_ack(self, command: int, timeout_s: float | None = None) -> CommandAck: ...

    def send_position_target_local_ned(
        self,
        *,
        frame: int,
        type_mask: int,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        afx: float = 0.0,
        afy: float = 0.0,
        afz: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None: ...

    def send_position_target_global_int(
        self,
        *,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
    ) -> None: ...

    def state_snapshot(self) -> VehicleState: ...


class MavSession:
    """真实 MAVLink 会话（串口或 UDP）。"""

    _NETWORK_PREFIXES = ("udp:", "udpin:", "udpout:", "tcp:", "tcpin:", "tcpout:")

    def __init__(self, config: MavlinkConfig, state_cache: VehicleStateCache, logger: logging.Logger | None = None) -> None:
        self._cfg = config
        self.state_cache = state_cache
        self._log = logger or logging.getLogger(__name__)

        self._master: Any | None = None
        self._connected = False
        self._recv_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._ack_lock = threading.Lock()
        self._ack_queues: dict[int, queue.Queue[Any]] = {}

        self.target_system: int | None = None
        self.target_component: int | None = None

    def connect(self) -> None:
        if self._connected:
            return
        if mavutil is None:
            raise RuntimeError("pymavlink 未安装，无法建立真实会话")

        retries = 0
        started = time.monotonic()
        while True:
            try:
                self._master = self._create_connection()
                heartbeat = self._master.wait_heartbeat(timeout=self._cfg.wait_heartbeat_timeout_s)
                if heartbeat is None:
                    raise TimeoutError("等待 heartbeat 超时")
                break
            except Exception as exc:
                retries += 1
                self._log.warning("connect attempt %s failed: %s", retries, exc)
                if self._cfg.max_retries > 0 and retries >= self._cfg.max_retries:
                    raise
                if self._cfg.connect_timeout_s > 0 and (time.monotonic() - started) >= self._cfg.connect_timeout_s:
                    raise TimeoutError("连接飞控超时") from exc
                time.sleep(self._cfg.retry_interval_s)

        assert self._master is not None
        assert mavutil is not None

        self.target_system = int(self._master.target_system)
        self.target_component = int(self._master.target_component)

        self.state_cache.update_heartbeat(
            mode=mavutil.mode_string_v10(heartbeat),
            armed=bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            system_status=int(heartbeat.system_status),
        )

        self._connected = True
        self.request_message_intervals()

        self._stop_event.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, name="mav-recv", daemon=True)
        self._recv_thread.start()

        self._log.info("MAVLink connected sys=%s comp=%s", self.target_system, self.target_component)

    def _create_connection(self):
        assert mavutil is not None
        link = self._cfg.port.strip().lower()
        kwargs: dict[str, Any] = {
            "source_system": self._cfg.source_system,
            "source_component": self._cfg.source_component,
            "autoreconnect": True,
        }
        if not link.startswith(self._NETWORK_PREFIXES):
            kwargs["baud"] = self._cfg.baud
        return mavutil.mavlink_connection(self._cfg.port, **kwargs)

    def close(self) -> None:
        self._stop_event.set()
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.5)
        self._recv_thread = None
        self._connected = False
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
        self._master = None

    def is_connected(self) -> bool:
        return self._connected and self._master is not None

    def state_snapshot(self) -> VehicleState:
        return self.state_cache.snapshot()

    def last_heartbeat_age_s(self) -> float:
        state = self.state_cache.snapshot()
        if state.last_heartbeat_monotonic is None:
            return math.inf
        return max(0.0, time.monotonic() - state.last_heartbeat_monotonic)

    def _ack_queue_for(self, command: int) -> queue.Queue[Any]:
        with self._ack_lock:
            if command not in self._ack_queues:
                self._ack_queues[command] = queue.Queue()
            return self._ack_queues[command]

    def wait_command_ack(self, command: int, timeout_s: float | None = None) -> CommandAck:
        queue_for_cmd = self._ack_queue_for(command)
        timeout = self._cfg.command_ack_timeout_s if timeout_s is None else timeout_s
        try:
            msg = queue_for_cmd.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError(f"等待 COMMAND_ACK 超时: command={command}") from exc
        return CommandAck(command=int(msg.command), result=int(msg.result))

    def command_long(
        self,
        command: int,
        params: Iterable[float],
        *,
        timeout_s: float | None = None,
        wait_ack: bool = True,
    ) -> CommandAck:
        if not self.is_connected():
            raise RuntimeError("session 未连接")
        assert self._master is not None
        p = list(params)
        if len(p) > 7:
            raise ValueError("command_long 参数不能超过 7 个")
        while len(p) < 7:
            p.append(0.0)

        queue_for_cmd = self._ack_queue_for(command)
        while not queue_for_cmd.empty():
            try:
                queue_for_cmd.get_nowait()
            except queue.Empty:
                break

        tries = 1 if not wait_ack else max(1, self._cfg.command_retries)
        timeout = self._cfg.command_ack_timeout_s if timeout_s is None else timeout_s
        last_error: Exception | None = None

        for attempt in range(1, tries + 1):
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                command,
                0,
                *p,
            )
            self._log.debug("COMMAND_LONG sent attempt=%s command=%s params=%s", attempt, command, p)
            if not wait_ack:
                return CommandAck(command=command, result=0)

            try:
                ack = self.wait_command_ack(command, timeout_s=timeout)
            except TimeoutError as exc:
                last_error = exc
                continue
            return ack

        if last_error is not None:
            raise last_error
        raise TimeoutError(f"等待 COMMAND_ACK 超时: command={command}")

    def set_mode(self, mode: str, *, timeout_s: float = 5.0) -> None:
        if not self.is_connected():
            raise RuntimeError("session 未连接")
        assert self._master is not None
        assert mavutil is not None

        wanted = mode.upper()
        mode_map = self._master.mode_mapping() or {}
        mode_id = None
        for name, value in mode_map.items():
            if str(name).upper() == wanted:
                mode_id = value
                break
        if mode_id is None:
            raise ValueError(f"飞控不支持模式: {mode}")

        self._master.mav.set_mode_send(
            self._master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        if timeout_s <= 0:
            return

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.state_cache.snapshot().mode.upper() == wanted:
                return
            time.sleep(0.1)
        raise TimeoutError(f"切模式超时: {mode}")

    def request_message_intervals(self) -> None:
        if not self.is_connected() or mavutil is None:
            return

        rates = self._cfg.request_rates
        msg_to_hz = {
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: rates.heartbeat_hz,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: rates.global_position_int_hz,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: rates.sys_status_hz,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: rates.gps_raw_int_hz,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: rates.rc_channels_hz,
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION: rates.home_position_hz,
        }
        for msg_id, hz in msg_to_hz.items():
            if hz <= 0:
                continue
            interval_us = int(1_000_000 / hz)
            try:
                self.command_long(
                    int(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL),
                    [float(msg_id), float(interval_us), 0, 0, 0, 0, 0],
                    wait_ack=False,
                )
            except Exception as exc:
                self._log.warning("request interval failed msg_id=%s: %s", msg_id, exc)

    def send_position_target_local_ned(
        self,
        *,
        frame: int,
        type_mask: int,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        afx: float = 0.0,
        afy: float = 0.0,
        afz: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        if not self.is_connected():
            raise RuntimeError("session 未连接")
        assert self._master is not None
        self._master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self._master.target_system,
            self._master.target_component,
            frame,
            type_mask,
            float(x),
            float(y),
            float(z),
            float(vx),
            float(vy),
            float(vz),
            float(afx),
            float(afy),
            float(afz),
            float(yaw),
            float(yaw_rate),
        )

    def send_position_target_global_int(self, *, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        if not self.is_connected():
            raise RuntimeError("session 未连接")
        assert self._master is not None
        assert mavutil is not None

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self._master.mav.set_position_target_global_int_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            self._master.target_system,
            self._master.target_component,
            int(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
            int(type_mask),
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            float(alt_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _recv_loop(self) -> None:
        assert self._master is not None
        while not self._stop_event.is_set():
            try:
                msg = self._master.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:
                self._log.warning("recv error: %s", exc)
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: Any) -> None:
        if mavutil is None:
            return
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            return

        if msg_type == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = bool(int(msg.base_mode) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            self.state_cache.update_heartbeat(mode=mode, armed=armed, system_status=int(msg.system_status))
            return

        if msg_type == "GLOBAL_POSITION_INT":
            self.state_cache.update_position(
                lat_deg=float(msg.lat) / 1e7,
                lon_deg=float(msg.lon) / 1e7,
                alt_m=float(msg.alt) / 1000.0,
                relative_alt_m=float(msg.relative_alt) / 1000.0,
                heading_deg=None if int(msg.hdg) == 65535 else float(msg.hdg) / 100.0,
            )
            self.state_cache.update_velocity(
                vx_mps=float(msg.vx) / 100.0,
                vy_mps=float(msg.vy) / 100.0,
                vz_mps=float(msg.vz) / 100.0,
            )
            return

        if msg_type == "SYS_STATUS":
            voltage = float(msg.voltage_battery) / 1000.0 if int(msg.voltage_battery) > 0 else None
            current = float(msg.current_battery) / 100.0 if int(msg.current_battery) >= 0 else None
            remaining = int(msg.battery_remaining) if int(msg.battery_remaining) >= 0 else None
            self.state_cache.update_battery(voltage_v=voltage, current_a=current, remaining_pct=remaining)
            return

        if msg_type == "GPS_RAW_INT":
            self.state_cache.update_gps(fix_type=int(msg.fix_type), satellites_visible=int(msg.satellites_visible))
            return

        if msg_type == "RC_CHANNELS":
            channels: dict[int, int] = {}
            for idx in range(1, 19):
                raw = getattr(msg, f"chan{idx}_raw", None)
                if raw is None:
                    continue
                value = int(raw)
                if value > 0:
                    channels[idx] = value
            self.state_cache.update_rc_channels(channels)
            return

        if msg_type == "HOME_POSITION":
            self.state_cache.update_home(
                lat_deg=float(msg.latitude) / 1e7,
                lon_deg=float(msg.longitude) / 1e7,
                alt_m=float(msg.altitude) / 1000.0,
            )
            return

        if msg_type == "STATUSTEXT":
            severity = int(getattr(msg, "severity", -1))
            raw_text = getattr(msg, "text", "")
            text = raw_text.decode("utf-8", errors="ignore") if isinstance(raw_text, bytes) else str(raw_text)
            text = text.replace("\x00", "").strip()
            if text:
                self.state_cache.update_status_text(severity=severity, text=text)
                self._log.info("STATUSTEXT[%s] %s", severity, text)
            return

        if msg_type == "EKF_STATUS_REPORT":
            flags = int(getattr(msg, "flags", 0))
            healthy = self._compute_ekf_health(flags)
            self.state_cache.update_ekf_health(ekf_healthy=healthy)
            return

        if msg_type == "COMMAND_ACK":
            queue_for_cmd = self._ack_queue_for(int(msg.command))
            queue_for_cmd.put(msg)
            return

    def _compute_ekf_health(self, flags: int) -> bool:
        if mavutil is None:
            return bool(flags)
        required = (
            int(getattr(mavutil.mavlink, "EKF_ATTITUDE", 1 << 0))
            | int(getattr(mavutil.mavlink, "EKF_VELOCITY_HORIZ", 1 << 1))
            | int(getattr(mavutil.mavlink, "EKF_POS_HORIZ_ABS", 1 << 3))
        )
        return (flags & required) == required


class FakeMavSession:
    """dry-run 会话，用于测试控制层和主状态机。"""

    _CMD_ARM_DISARM = 400
    _CMD_NAV_TAKEOFF = 22

    def __init__(self, config: MavlinkConfig, state_cache: VehicleStateCache, logger: logging.Logger | None = None) -> None:
        self._cfg = config
        self.state_cache = state_cache
        self._log = logger or logging.getLogger(__name__)
        self._connected = False
        self._ack_queues: dict[int, queue.Queue[CommandAck]] = {}
        self.command_history: list[tuple[int, list[float], bool]] = []
        self.velocity_history: list[tuple[float, float, float, float]] = []
        self.goto_history: list[tuple[float, float, float]] = []

        self.target_system = 1
        self.target_component = 1
        self.auto_heartbeat = True

    def connect(self) -> None:
        self._connected = True
        self.state_cache.update_heartbeat(mode="STANDBY", armed=False, system_status=3)
        self.state_cache.update_position(lat_deg=30.0, lon_deg=120.0, alt_m=0.0, relative_alt_m=0.0, heading_deg=0.0)
        self.state_cache.update_velocity(vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
        self.state_cache.update_gps(fix_type=3, satellites_visible=12)
        self.state_cache.update_ekf_health(ekf_healthy=True)
        self.state_cache.update_battery(voltage_v=15.8, current_a=0.0, remaining_pct=80)
        self.state_cache.update_rc_channels({1: 1500, 2: 1500, 3: 1000, 4: 1500, 5: 1800, 7: 1000, 8: 1000})
        self.state_cache.update_home(lat_deg=30.0, lon_deg=120.0, alt_m=0.0)
        self.state_cache.update_status_text(severity=6, text="Fake session connected")
        self._log.info("FakeMavSession connected")

    def close(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def state_snapshot(self) -> VehicleState:
        return self.state_cache.snapshot()

    def last_heartbeat_age_s(self) -> float:
        if self._connected and self.auto_heartbeat:
            state = self.state_cache.snapshot()
            if (
                state.last_heartbeat_monotonic is None
                or time.monotonic() - state.last_heartbeat_monotonic > 0.2
            ):
                self._touch_heartbeat()
        state = self.state_cache.snapshot()
        if state.last_heartbeat_monotonic is None:
            return math.inf
        return max(0.0, time.monotonic() - state.last_heartbeat_monotonic)

    def _queue_for_cmd(self, command: int) -> queue.Queue[CommandAck]:
        if command not in self._ack_queues:
            self._ack_queues[command] = queue.Queue()
        return self._ack_queues[command]

    def wait_command_ack(self, command: int, timeout_s: float | None = None) -> CommandAck:
        timeout = self._cfg.command_ack_timeout_s if timeout_s is None else timeout_s
        q = self._queue_for_cmd(command)
        try:
            return q.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError(f"fake ack timeout command={command}") from exc

    def command_long(
        self,
        command: int,
        params: Iterable[float],
        *,
        timeout_s: float | None = None,
        wait_ack: bool = True,
    ) -> CommandAck:
        del timeout_s
        if not self.is_connected():
            raise RuntimeError("fake session 未连接")

        p = list(params)
        while len(p) < 7:
            p.append(0.0)
        self.command_history.append((command, p, wait_ack))

        state = self.state_cache.snapshot()
        if command == self._CMD_ARM_DISARM:
            self.state_cache.update_heartbeat(mode=state.mode, armed=bool(p[0] >= 0.5), system_status=state.system_status)
        elif command == self._CMD_NAV_TAKEOFF:
            target_alt = max(0.2, float(p[6]))
            self.state_cache.update_position(
                lat_deg=state.position.lat_deg,
                lon_deg=state.position.lon_deg,
                alt_m=target_alt,
                relative_alt_m=target_alt,
                heading_deg=state.position.heading_deg,
            )
            self.state_cache.update_velocity(vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)

        ack = CommandAck(command=command, result=0)
        self._queue_for_cmd(command).put(ack)
        self._touch_heartbeat()
        return ack if wait_ack else CommandAck(command=command, result=0)

    def set_mode(self, mode: str, *, timeout_s: float = 5.0) -> None:
        del timeout_s
        state = self.state_cache.snapshot()
        wanted = mode.upper()
        armed = state.armed
        if wanted == "LAND":
            armed = False
            self.state_cache.update_position(
                lat_deg=state.position.lat_deg,
                lon_deg=state.position.lon_deg,
                alt_m=0.0,
                relative_alt_m=0.0,
                heading_deg=state.position.heading_deg,
            )
        self.state_cache.update_heartbeat(mode=wanted, armed=armed, system_status=state.system_status)

    def request_message_intervals(self) -> None:
        return

    def send_position_target_local_ned(
        self,
        *,
        frame: int,
        type_mask: int,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        afx: float = 0.0,
        afy: float = 0.0,
        afz: float = 0.0,
        yaw: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        del frame, type_mask, x, y, z, afx, afy, afz, yaw
        self.velocity_history.append((float(vx), float(vy), float(vz), float(yaw_rate)))
        self.state_cache.update_velocity(vx_mps=float(vx), vy_mps=float(vy), vz_mps=float(vz))
        self._touch_heartbeat()

    def send_position_target_global_int(self, *, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        self.goto_history.append((lat_deg, lon_deg, alt_m))
        state = self.state_cache.snapshot()
        self.state_cache.update_position(
            lat_deg=float(lat_deg),
            lon_deg=float(lon_deg),
            alt_m=float(alt_m),
            relative_alt_m=float(alt_m),
            heading_deg=state.position.heading_deg,
        )
        self._touch_heartbeat()

    def _touch_heartbeat(self) -> None:
        state = self.state_cache.snapshot()
        self.state_cache.update_heartbeat(mode=state.mode, armed=state.armed, system_status=state.system_status)
        if state.rc.channels:
            self.state_cache.update_rc_channels(state.rc.channels)
