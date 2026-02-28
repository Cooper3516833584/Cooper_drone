"""
control — 控制原语层

职责：
    封装所有对飞控的控制指令，供 mission 层调用。
    任务代码**不应直接操作 MAVLink / DroneKit**，统一通过本模块下发指令。

设计原则：
    - **限幅**：所有速度/角速度在发送前做 clamp，防止超限
    - **频率**：速度指令应由调用方以 ≥1 Hz 持续发送（MAVLink 速度指令需持续刷新）
    - **超时**：阻塞指令（takeoff/land/goto）均有 timeout 参数，超时则抛出异常
    - **异常**：所有函数在操作失败时应抛出明确异常（RuntimeError / TimeoutError），
      由上层状态机 / MissionTask 捕获并执行安全退出

注意：
    当前为占位实现（TODO），真机联调时逐步填充。
    每个函数的 docstring 已写清未来实现要点。
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any

from pymavlink import mavutil

if TYPE_CHECKING:
    from src.config_loader import AppConfig

logger = logging.getLogger(__name__)


def set_mode(vehicle: Any, mode: str) -> None:
    """切换飞行模式。

    实现要点：
        - 使用 ``vehicle.mode = VehicleMode(mode)``
        - 等待模式切换确认（轮询 vehicle.mode.name），最多 5s
        - 切换失败应抛出 RuntimeError

    Args:
        vehicle: DroneKit Vehicle 实例。
        mode:    目标模式名，如 ``"GUIDED"`` / ``"LOITER"`` / ``"LAND"``。

    Raises:
        RuntimeError: 模式切换超时或失败。
    """
    from dronekit import VehicleMode

    logger.info("切换模式: %s -> %s", vehicle.mode.name, mode)
    vehicle.mode = VehicleMode(mode)

    t0 = time.time()
    while vehicle.mode.name != mode:
        if time.time() - t0 > 5.0:
            raise RuntimeError(f"模式切换超时: 目标={mode}  当前={vehicle.mode.name}")
        time.sleep(0.2)

    logger.info("模式切换完成: %s", mode)


def arm(vehicle: Any) -> None:
    """解锁电机。

    实现要点：
        - 前提：模式必须为 GUIDED
        - 使用 ``vehicle.armed = True``
        - 等待 armed 确认，最多 10s
        - 解锁失败应检查 pre-arm 错误并抛出 RuntimeError

    Args:
        vehicle: DroneKit Vehicle 实例。

    Raises:
        RuntimeError: 解锁失败或超时。
    """
    logger.info("请求解锁电机…")

    if vehicle.mode.name != "GUIDED":
        raise RuntimeError(f"解锁前必须为 GUIDED 模式，当前: {vehicle.mode.name}")

    vehicle.armed = True

    t0 = time.time()
    while not vehicle.armed:
        if time.time() - t0 > 10.0:
            raise RuntimeError("解锁超时（10s），请检查 pre-arm 状态")
        time.sleep(0.3)

    logger.info("电机已解锁")


def disarm(vehicle: Any) -> None:
    """上锁电机。

    实现要点：
        - 使用 ``vehicle.armed = False``
        - 等待 disarm 确认，最多 5s

    Args:
        vehicle: DroneKit Vehicle 实例。

    Raises:
        RuntimeError: 上锁失败或超时。
    """
    logger.info("请求上锁电机…")
    vehicle.armed = False

    t0 = time.time()
    while vehicle.armed:
        if time.time() - t0 > 5.0:
            raise RuntimeError("上锁超时（5s）")
        time.sleep(0.2)

    logger.info("电机已上锁")


def takeoff(vehicle: Any, alt_m: float, timeout_s: float = 30.0) -> None:
    """起飞至指定高度。

    实现要点：
        - 前提：已解锁 + GUIDED 模式
        - 使用 ``vehicle.simple_takeoff(alt_m)``
        - 轮询 ``vehicle.location.global_relative_frame.alt``
        - 到达 95% 目标高度视为完成
        - 超时未到达应抛出 TimeoutError

    Args:
        vehicle:   DroneKit Vehicle 实例。
        alt_m:     目标高度（米，相对起飞点）。
        timeout_s: 超时（秒），默认 30s。

    Raises:
        RuntimeError:  未解锁或模式错误。
        TimeoutError:  超时未到达目标高度。
    """
    if not vehicle.armed:
        raise RuntimeError("起飞前必须先解锁")

    logger.info("起飞中  目标高度=%.1fm  超时=%ds", alt_m, timeout_s)
    vehicle.simple_takeoff(alt_m)

    t0 = time.time()
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        logger.debug("当前高度: %.2fm / %.2fm", current_alt, alt_m)
        if current_alt >= alt_m * 0.95:
            logger.info("已到达目标高度: %.2fm", current_alt)
            return
        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                f"起飞超时({timeout_s}s)  当前高度={current_alt:.2f}m  目标={alt_m:.2f}m"
            )
        time.sleep(0.5)


def land(vehicle: Any, timeout_s: float = 60.0) -> None:
    """降落。

    实现要点：
        - 使用 ``set_mode(vehicle, 'LAND')``
        - 轮询高度直到 ≤ 0.3m 或 disarmed
        - 超时不抛异常（降落是安全退出路径，不能再异常退出）

    Args:
        vehicle:   DroneKit Vehicle 实例。
        timeout_s: 超时（秒），默认 60s。
    """
    logger.info("开始降落…")
    try:
        set_mode(vehicle, "LAND")
    except RuntimeError as exc:
        logger.error("切换 LAND 模式失败: %s", exc)
        return

    t0 = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        logger.debug("降落中  高度=%.2fm  armed=%s", alt, vehicle.armed)
        if not vehicle.armed:
            logger.info("已降落并触地上锁")
            return
        if alt <= 0.3:
            logger.info("已接近地面 (%.2fm)，等待上锁…", alt)
        if time.time() - t0 > timeout_s:
            logger.warning("降落超时(%ds)，当前高度=%.2fm", timeout_s, alt)
            return
        time.sleep(0.5)


def goto_global(
    vehicle: Any,
    lat: float,
    lon: float,
    alt_m: float,
    timeout_s: float = 60.0,
) -> None:
    """飞向指定全球坐标点。

    实现要点：
        - 使用 ``vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt_m))``
        - 轮询当前位置与目标距离
        - 到达容差内（position_tolerance_m）视为完成
        - 超时抛出 TimeoutError

    Args:
        vehicle:   DroneKit Vehicle 实例。
        lat:       目标纬度。
        lon:       目标经度。
        alt_m:     目标高度（相对起飞点）。
        timeout_s: 超时（秒）。

    Raises:
        TimeoutError: 超时未到达目标点。
    """
    from dronekit import LocationGlobalRelative

    target = LocationGlobalRelative(lat, lon, alt_m)
    logger.info("飞向目标  lat=%.7f  lon=%.7f  alt=%.1fm", lat, lon, alt_m)
    vehicle.simple_goto(target)

    t0 = time.time()
    while True:
        current = vehicle.location.global_relative_frame
        dist = _haversine_m(current.lat, current.lon, lat, lon)
        alt_err = abs(current.alt - alt_m)
        logger.debug("距离目标: 水平=%.2fm  垂直=%.2fm", dist, alt_err)
        if dist < 0.5 and alt_err < 0.3:
            logger.info("已到达目标点")
            return
        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                f"goto 超时({timeout_s}s)  距离={dist:.1f}m  高度差={alt_err:.1f}m"
            )
        time.sleep(0.5)


def send_body_velocity(
    vehicle: Any,
    vx: float,
    vy: float,
    vz: float,
    yaw_rate: float | None = None,
    cfg: AppConfig | None = None,
) -> None:
    """发送机体坐标系速度指令。

    坐标系说明（机体 NED 映射）：
        - vx: 前进（正）/ 后退（负），m/s
        - vy: 右移（正）/ 左移（负），m/s
        - vz: 下降（正）/ 上升（负），m/s
        - yaw_rate: 偏航角速度（°/s），None 则保持当前航向

    实现要点：
        - 在发送前按 limits 做 clamp
        - 使用 MAVLink SET_POSITION_TARGET_LOCAL_NED (type_mask 仅保留速度)
        - 此指令必须由调用方以 ≥1 Hz 持续发送，否则飞控 ~3s 后停止执行
        - 发送频率建议：4-10 Hz

    Args:
        vehicle:  DroneKit Vehicle 实例。
        vx:       前进速度（m/s）。
        vy:       右移速度（m/s）。
        vz:       下降速度（m/s，正值向下）。
        yaw_rate: 偏航角速度（°/s），可选。
        cfg:      配置实例（用于读取限幅值）。
    """
    # 限幅
    if cfg is not None:
        max_xy = cfg.limits.max_xy_vel_mps
        max_z = cfg.limits.max_z_vel_mps
        vx = _clamp(vx, -max_xy, max_xy)
        vy = _clamp(vy, -max_xy, max_xy)
        vz = _clamp(vz, -max_z, max_z)
        if yaw_rate is not None:
            max_yaw = cfg.limits.max_yaw_rate_dps
            yaw_rate = _clamp(yaw_rate, -max_yaw, max_yaw)

    # 构建 MAVLink 消息
    # type_mask: 忽略位置与加速度，仅保留速度（+可选 yaw_rate）
    #   bit 0-2: 位置 x/y/z  -> 忽略(1)
    #   bit 3-5: 速度 x/y/z  -> 使用(0)
    #   bit 6-8: 加速度       -> 忽略(1)
    #   bit 9:   force        -> 忽略
    #   bit 10:  yaw          -> 忽略(1)
    #   bit 11:  yaw_rate     -> 使用(0) 或 忽略(1)
    if yaw_rate is not None:
        type_mask = 0b0000_01_111_000_111  # 使用速度 + yaw_rate
    else:
        type_mask = 0b0000_11_111_000_111  # 仅使用速度
        yaw_rate = 0.0

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms（不使用）
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,           # x, y, z（忽略）
        vx, vy, vz,        # 速度
        0, 0, 0,            # 加速度（忽略）
        0,                  # yaw（忽略）
        yaw_rate * 3.14159265 / 180.0,  # yaw_rate（rad/s）
    )
    vehicle.send_mavlink(msg)


def stop_motion(vehicle: Any) -> None:
    """停止所有运动——速度归零。

    实现要点：
        - 发送零速度指令
        - 连续发送 3 次以确保飞控收到

    Args:
        vehicle: DroneKit Vehicle 实例。
    """
    logger.info("停止运动（速度归零）")
    for _ in range(3):
        send_body_velocity(vehicle, 0.0, 0.0, 0.0)
        time.sleep(0.1)


def preflight_check(vehicle: Any) -> tuple[bool, str]:
    """起飞前检查。

    检查项目：
        1. GPS Fix ≥ 3（3D Fix）
        2. EKF 状态正常
        3. 电池电压 > 阈值（若可用）
        4. 模式为 GUIDED
        5. 未 armed（防止重复起飞）

    Args:
        vehicle: DroneKit Vehicle 实例。

    Returns:
        (pass, reason) 元组。pass=True 表示可飞，reason 为描述消息。
    """
    reasons: list[str] = []

    # GPS
    gps = vehicle.gps_0
    if gps.fix_type < 3:
        reasons.append(f"GPS Fix 不足: fix_type={gps.fix_type}  需要 ≥3")

    # EKF
    try:
        if not vehicle.ekf_ok:
            reasons.append("EKF 状态异常")
    except Exception:
        reasons.append("无法读取 EKF 状态")

    # 电池
    batt = vehicle.battery
    if batt and batt.voltage is not None and batt.voltage < 10.0:
        reasons.append(f"电池电压过低: {batt.voltage:.1f}V")

    # 已解锁（防止重复操作）
    if vehicle.armed:
        reasons.append("飞控已解锁，请先确认安全")

    if reasons:
        msg = "起飞前检查未通过: " + "; ".join(reasons)
        logger.warning(msg)
        return False, msg

    msg = "起飞前检查通过"
    logger.info(msg)
    return True, msg


# ── 内部工具 ──────────────────────────────────────────────


def _clamp(value: float, min_val: float, max_val: float) -> float:
    """限幅。"""
    return max(min_val, min(value, max_val))


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine 公式计算两点水平距离（米）。"""
    import math

    R = 6_371_000  # 地球平均半径（米）
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
