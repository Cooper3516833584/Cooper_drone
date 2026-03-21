from __future__ import annotations

import math
import threading
import time

from src.config_loader import AppConfig, LimitsConfig
from src.mav_session import SessionLike, mavutil

_CMD_COMPONENT_ARM_DISARM = 400
_CMD_NAV_TAKEOFF = 22
_FORCE_DISARM_MAGIC = 21196

_CONTROL_LOCK = threading.RLock()
_INHIBIT_LOCK = threading.RLock()
_MOTION_INHIBIT = False
_MOTION_INHIBIT_REASON = ""
_RUNTIME_LIMITS = LimitsConfig()

if mavutil is not None:
    _FRAME_BODY_NED = int(mavutil.mavlink.MAV_FRAME_BODY_NED)
    _MASK_X_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
    _MASK_Y_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
    _MASK_Z_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
    _MASK_AX_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE)
    _MASK_AY_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE)
    _MASK_AZ_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
    _MASK_YAW_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
    _MASK_YAW_RATE_IGNORE = int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
else:
    _FRAME_BODY_NED = 8
    _MASK_X_IGNORE = 1 << 0
    _MASK_Y_IGNORE = 1 << 1
    _MASK_Z_IGNORE = 1 << 2
    _MASK_AX_IGNORE = 1 << 6
    _MASK_AY_IGNORE = 1 << 7
    _MASK_AZ_IGNORE = 1 << 8
    _MASK_YAW_IGNORE = 1 << 10
    _MASK_YAW_RATE_IGNORE = 1 << 11


def bind_runtime_limits(limits: LimitsConfig) -> None:
    global _RUNTIME_LIMITS
    _RUNTIME_LIMITS = limits


def _accepted_results() -> set[int]:
    if mavutil is None:
        return {0, 1}
    return {
        int(mavutil.mavlink.MAV_RESULT_ACCEPTED),
        int(mavutil.mavlink.MAV_RESULT_IN_PROGRESS),
    }


def _check_ack(ack) -> None:
    if int(ack.result) not in _accepted_results():
        raise RuntimeError(f"命令被拒绝: command={ack.command} result={ack.result}")


def _wait_until(predicate, timeout_s: float, message: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(message)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _safe_mode_name(mode: str) -> str:
    return mode.strip().upper()


def _ensure_motion_allowed() -> None:
    if motion_output_inhibited():
        raise RuntimeError(f"motion output inhibited: {_MOTION_INHIBIT_REASON}")


def inhibit_motion_outputs(reason: str = "") -> None:
    global _MOTION_INHIBIT, _MOTION_INHIBIT_REASON
    with _INHIBIT_LOCK:
        _MOTION_INHIBIT = True
        _MOTION_INHIBIT_REASON = reason.strip() or "inhibited_by_safety"


def clear_motion_inhibit() -> None:
    global _MOTION_INHIBIT, _MOTION_INHIBIT_REASON
    with _INHIBIT_LOCK:
        _MOTION_INHIBIT = False
        _MOTION_INHIBIT_REASON = ""


def motion_output_inhibited() -> bool:
    with _INHIBIT_LOCK:
        return _MOTION_INHIBIT


def set_mode(session: SessionLike, mode: str) -> None:
    with _CONTROL_LOCK:
        session.set_mode(_safe_mode_name(mode), timeout_s=5.0)


def set_mode_nowait(session: SessionLike, mode: str) -> None:
    with _CONTROL_LOCK:
        session.set_mode(_safe_mode_name(mode), timeout_s=0.0)


def arm(session: SessionLike, *, timeout_s: float = 6.0) -> None:
    with _CONTROL_LOCK:
        ack = session.command_long(
            _CMD_COMPONENT_ARM_DISARM,
            [1, 0, 0, 0, 0, 0, 0],
            timeout_s=timeout_s,
            wait_ack=True,
        )
        _check_ack(ack)
        _wait_until(lambda: session.state_snapshot().armed, timeout_s, "arm 状态确认超时")


def disarm(session: SessionLike, *, timeout_s: float = 6.0) -> None:
    with _CONTROL_LOCK:
        ack = session.command_long(
            _CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],
            timeout_s=timeout_s,
            wait_ack=True,
        )
        _check_ack(ack)
        _wait_until(lambda: not session.state_snapshot().armed, timeout_s, "disarm 状态确认超时")


def disarm_nowait(session: SessionLike) -> None:
    with _CONTROL_LOCK:
        session.command_long(_CMD_COMPONENT_ARM_DISARM, [0, 0, 0, 0, 0, 0, 0], wait_ack=False)


def force_disarm(session: SessionLike, *, timeout_s: float = 4.0) -> None:
    with _CONTROL_LOCK:
        ack = session.command_long(
            _CMD_COMPONENT_ARM_DISARM,
            [0, _FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0],
            timeout_s=timeout_s,
            wait_ack=True,
        )
        _check_ack(ack)
        _wait_until(lambda: not session.state_snapshot().armed, timeout_s, "force_disarm 状态确认超时")


def takeoff(session: SessionLike, alt_m: float, timeout_s: float = 30.0) -> None:
    target_alt = max(0.2, float(alt_m))
    with _CONTROL_LOCK:
        _ensure_motion_allowed()
        state = session.state_snapshot()
        if state.mode.upper() != "GUIDED":
            session.set_mode("GUIDED", timeout_s=5.0)
        if not session.state_snapshot().armed:
            ack = session.command_long(_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0], wait_ack=True)
            _check_ack(ack)
            _wait_until(lambda: session.state_snapshot().armed, 6.0, "自动 arm 超时")

        ack = session.command_long(_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, target_alt], wait_ack=True)
        _check_ack(ack)

    _wait_until(
        lambda: (session.state_snapshot().position.relative_alt_m or 0.0) >= (target_alt - _RUNTIME_LIMITS.altitude_accept_error_m),
        timeout_s,
        f"起飞超时，目标高度 {target_alt:.2f}m",
    )


def land(session: SessionLike, timeout_s: float = 30.0) -> None:
    with _CONTROL_LOCK:
        session.set_mode("LAND", timeout_s=5.0)
    _wait_until(lambda: not session.state_snapshot().armed, timeout_s, "降落超时，armed 仍为 True")


def land_nowait(session: SessionLike) -> None:
    with _CONTROL_LOCK:
        session.set_mode("LAND", timeout_s=0.0)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return r * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def goto_global(session: SessionLike, lat: float, lon: float, alt_m: float, timeout_s: float = 0.0) -> None:
    with _CONTROL_LOCK:
        _ensure_motion_allowed()
        session.send_position_target_global_int(lat_deg=lat, lon_deg=lon, alt_m=alt_m)

    if timeout_s <= 0:
        return

    def _arrived() -> bool:
        s = session.state_snapshot()
        if s.position.lat_deg is None or s.position.lon_deg is None:
            return False
        distance = _haversine_m(s.position.lat_deg, s.position.lon_deg, lat, lon)
        alt_ok = s.position.relative_alt_m is not None and abs(s.position.relative_alt_m - alt_m) <= _RUNTIME_LIMITS.altitude_accept_error_m
        return distance <= _RUNTIME_LIMITS.goto_accept_radius_m and alt_ok

    _wait_until(_arrived, timeout_s, "goto_global 超时未到达接受半径")


def send_body_velocity(
    session: SessionLike,
    vx: float,
    vy: float,
    vz: float,
    yaw_rate: float | None = None,
) -> None:
    with _CONTROL_LOCK:
        _ensure_motion_allowed()
        _send_body_velocity_raw(session, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)


def _send_body_velocity_raw(
    session: SessionLike,
    *,
    vx: float,
    vy: float,
    vz: float,
    yaw_rate: float | None,
) -> None:
    with _CONTROL_LOCK:
        max_xy = _RUNTIME_LIMITS.max_xy_vel_mps
        max_z = _RUNTIME_LIMITS.max_z_vel_mps
        max_yaw = _RUNTIME_LIMITS.max_yaw_rate_dps

        vx_limited = _clamp(float(vx), -max_xy, max_xy)
        vy_limited = _clamp(float(vy), -max_xy, max_xy)
        vz_limited = _clamp(float(vz), -max_z, max_z)

        if yaw_rate is None:
            yaw_rate_dps = 0.0
            yaw_rate_enabled = False
        else:
            yaw_rate_dps = _clamp(float(yaw_rate), -max_yaw, max_yaw)
            yaw_rate_enabled = True

        type_mask = (
            _MASK_X_IGNORE
            | _MASK_Y_IGNORE
            | _MASK_Z_IGNORE
            | _MASK_AX_IGNORE
            | _MASK_AY_IGNORE
            | _MASK_AZ_IGNORE
            | _MASK_YAW_IGNORE
        )
        if not yaw_rate_enabled:
            type_mask |= _MASK_YAW_RATE_IGNORE

        session.send_position_target_local_ned(
            frame=_FRAME_BODY_NED,
            type_mask=int(type_mask),
            vx=vx_limited,
            vy=vy_limited,
            vz=vz_limited,
            yaw_rate=math.radians(yaw_rate_dps),
        )


def stop_motion(session: SessionLike) -> None:
    for _ in range(3):
        try:
            _send_body_velocity_raw(session, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
        except RuntimeError:
            # 被 inhibit 时也允许结束，不再抛出。
            pass
        time.sleep(0.05)


def preflight_check(session: SessionLike, cfg: AppConfig) -> None:
    bind_runtime_limits(cfg.limits)
    if not session.is_connected():
        raise RuntimeError("session 未连接")

    hb_age = session.last_heartbeat_age_s()
    if math.isinf(hb_age) or hb_age > cfg.mavlink.heartbeat_timeout_s:
        raise RuntimeError(f"heartbeat 超时: {hb_age:.2f}s")

    state = session.state_snapshot()
    if not state.mode or state.mode.upper() == "UNKNOWN":
        raise RuntimeError("模式不可读")

    if state.gps_fix_type is None:
        raise RuntimeError("GPS fix 不可读")
    if state.gps_fix_type < cfg.limits.min_gps_fix_type:
        raise RuntimeError(f"GPS fix 不达标: {state.gps_fix_type} < {cfg.limits.min_gps_fix_type}")

    if state.position.lat_deg is None or state.position.lon_deg is None:
        raise RuntimeError("位置数据缺失")
    if state.position.relative_alt_m is None:
        raise RuntimeError("相对高度数据缺失")

    if state.battery.voltage_v is None or state.battery.remaining_pct is None:
        raise RuntimeError("电池数据缺失")
    if state.battery.voltage_v < cfg.limits.min_battery_voltage_v:
        raise RuntimeError(f"电池电压过低: {state.battery.voltage_v}V < {cfg.limits.min_battery_voltage_v}V")
    if state.battery.remaining_pct < cfg.limits.min_battery_percent:
        raise RuntimeError(f"电池电量过低: {state.battery.remaining_pct}% < {cfg.limits.min_battery_percent}%")

    if not state.rc.channels:
        raise RuntimeError("RC 通道数据缺失")
    if state.rc.last_update_monotonic is None:
        raise RuntimeError("RC 更新时间缺失")
    rc_age = time.monotonic() - state.rc.last_update_monotonic
    if rc_age > cfg.rc.stale_timeout_s:
        raise RuntimeError(f"RC 数据过期: age={rc_age:.2f}s")

    mode_pwm = state.rc.channels.get(cfg.rc.mode_channel)
    if mode_pwm is None:
        raise RuntimeError(f"RC 模式通道缺失: channel={cfg.rc.mode_channel}")
    if mode_pwm < cfg.rc.guided_pwm_threshold:
        raise RuntimeError(
            f"RC 未授权 GUIDED 接管: value={mode_pwm}, threshold={cfg.rc.guided_pwm_threshold}"
        )

    if state.ekf_healthy is False:
        raise RuntimeError("EKF 不健康")

    if cfg.safety.check_home_position:
        if state.home_lat_deg is None or state.home_lon_deg is None:
            raise RuntimeError("home position 缺失")
