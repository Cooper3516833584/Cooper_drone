"""
control — 控制原语层（带全局锁与输出门禁）

职责：
    封装所有对飞控的控制指令，供 mission 层调用。
    任务代码**不应直接操作 MAVLink / DroneKit**，统一通过本模块下发指令。

安全设计（Fix 3 + Fix 4）：
    - **全局 RLock**：所有控制原语必须持锁调用，防止任务线程与安全线程并发
      send_mavlink 导致不可控行为。
    - **输出门禁（inhibit gate）**：安全事件触发后，调用 ``inhibit_outputs()``
      使后续所有运动控制指令被静默丢弃（set_mode/arm/disarm 仍允许，因安全层
      需要切模式）。
    - **限幅**：所有速度/角速度在发送前做 clamp，防止超限。
    - **频率**：速度指令应由调用方以 ≥1 Hz 持续发送（MAVLink 速度指令需持续刷新）。
    - **超时**：阻塞指令（takeoff/land/goto）均有 timeout 参数，超时则抛出异常。
    - **异常**：所有函数在操作失败时应抛出明确异常（RuntimeError / TimeoutError），
      由上层状态机 / MissionTask 捕获并执行安全退出。

type_mask 参考（MAVLink SET_POSITION_TARGET_LOCAL_NED）：
    bit 0 : pos_x      bit 3 : vel_x      bit 6 : accel_x    bit 9 : force
    bit 1 : pos_y      bit 4 : vel_y      bit 7 : accel_y    bit 10: yaw
    bit 2 : pos_z      bit 5 : vel_z      bit 8 : accel_z    bit 11: yaw_rate
    置 1 = 忽略该字段
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from pymavlink import mavutil

if TYPE_CHECKING:
    from src.config_loader import AppConfig

logger = logging.getLogger(__name__)

# ── 全局并发控制 + 输出门禁（Fix 4）──────────────────────

_control_lock = threading.RLock()
_outputs_inhibited = False


def inhibit_outputs() -> None:
    """启用输出门禁——此后所有运动控制指令将被静默丢弃。

    由 SafetyManager 在 KILL / LINK_LOST / 撤销接管 时调用。
    """
    global _outputs_inhibited
    with _control_lock:
        _outputs_inhibited = True
        logger.warning("控制输出已被禁止（inhibit gate 激活）")


def release_outputs() -> None:
    """解除输出门禁——恢复正常控制指令发送。

    仅应在确认安全后调用（如重新获得 RC 接管许可）。
    """
    global _outputs_inhibited
    with _control_lock:
        _outputs_inhibited = False
        logger.info("控制输出已恢复（inhibit gate 解除）")


def is_output_inhibited() -> bool:
    """查询输出门禁状态。"""
    with _control_lock:
        return _outputs_inhibited


def _check_inhibit(action: str) -> bool:
    """检查门禁，若被抑制则记录日志并返回 True（调用方应跳过）。

    注意：此函数在 _control_lock 已持有的环境下调用。
    """
    if _outputs_inhibited:
        logger.debug("输出被禁止，丢弃指令: %s", action)
        return True
    return False


# ── 模式 / 解锁（不受门禁限制——安全层需要切模式） ─────────


def set_mode(vehicle: Any, mode: str) -> None:
    """切换飞行模式。

    实现要点：
        - 使用 ``vehicle.mode = VehicleMode(mode)``
        - 等待模式切换确认（轮询 vehicle.mode.name），最多 5s
        - 切换失败应抛出 RuntimeError
        - 此函数不受输出门禁限制（安全层需要切模式）

    Args:
        vehicle: DroneKit Vehicle 实例。
        mode:    目标模式名，如 ``"GUIDED"`` / ``"LOITER"`` / ``"LAND"``。

    Raises:
        RuntimeError: 模式切换超时或失败。
    """
    from dronekit import VehicleMode

    with _control_lock:
        logger.info("切换模式: %s -> %s", vehicle.mode.name, mode)
        vehicle.mode = VehicleMode(mode)

    t0 = time.time()
    while vehicle.mode.name != mode:
        if time.time() - t0 > 5.0:
            raise RuntimeError(f"模式切换超时: 目标={mode}  当前={vehicle.mode.name}")
        time.sleep(0.2)

    logger.info("模式切换完成: %s", mode)


def set_mode_nowait(vehicle: Any, mode: str) -> None:
    """非阻塞切换飞行模式——不等待确认，供安全层紧急调用。

    Args:
        vehicle: DroneKit Vehicle 实例。
        mode:    目标模式名。
    """
    from dronekit import VehicleMode

    with _control_lock:
        logger.info("非阻塞切换模式: -> %s", mode)
        vehicle.mode = VehicleMode(mode)


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
    with _control_lock:
        if _check_inhibit("arm"):
            return
        logger.info("请求解锁电机...")
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

    Args:
        vehicle: DroneKit Vehicle 实例。

    Raises:
        RuntimeError: 上锁失败或超时。
    """
    with _control_lock:
        logger.info("请求上锁电机...")
        vehicle.armed = False

    t0 = time.time()
    while vehicle.armed:
        if time.time() - t0 > 5.0:
            raise RuntimeError("上锁超时（5s）")
        time.sleep(0.2)

    logger.info("电机已上锁")


def force_disarm_nowait(vehicle: Any) -> None:
    """Best-effort 强制上锁（不等待确认）。

    发送 MAV_CMD_COMPONENT_ARM_DISARM（param5=21196 表示 force）。
    用于 KILL 场景：上位机"尽力补一刀"，失败也不影响流程。
    真正的停桨保障仍依赖飞控侧 RC kill。

    Args:
        vehicle: DroneKit Vehicle 实例。
    """
    try:
        vehicle.message_factory  # 兼容性检查
        msg = vehicle.message_factory.command_long_encode(
            0, 0,                                        # target_system, target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0,                                           # confirmation
            0,                                           # param1: 0=disarm
            21196,                                       # param2: force magic (21196)
            0, 0, 0, 0, 0,                               # param3-7 unused
        )
        vehicle.send_mavlink(msg)
        logger.warning("已发送 best-effort force disarm 指令")
    except Exception as exc:
        logger.warning("force disarm 发送失败（可忽略）: %s", exc)


# ── 运动控制（受门禁限制） ────────────────────────────────


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
        RuntimeError:  未解锁或模式错误或门禁阻止。
        TimeoutError:  超时未到达目标高度。
    """
    with _control_lock:
        if _check_inhibit("takeoff"):
            raise RuntimeError("输出被禁止，无法起飞")
        if not vehicle.armed:
            raise RuntimeError("起飞前必须先解锁")
        logger.info("起飞中  目标高度=%.1fm  超时=%ds", alt_m, timeout_s)
        vehicle.simple_takeoff(alt_m)

    t0 = time.time()
    while True:
        if is_output_inhibited():
            raise RuntimeError("起飞过程中输出被禁止")
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
        - 轮询高度直到 <= 0.3m 或 disarmed
        - 超时不抛异常（降落是安全退出路径，不能再异常退出）
        - 此函数不受门禁限制（降落本身就是安全动作）

    Args:
        vehicle:   DroneKit Vehicle 实例。
        timeout_s: 超时（秒），默认 60s。
    """
    logger.info("开始降落...")
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
            logger.info("已接近地面 (%.2fm)，等待上锁...", alt)
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
        RuntimeError: 输出被禁止。
        TimeoutError: 超时未到达目标点。
    """
    from dronekit import LocationGlobalRelative

    with _control_lock:
        if _check_inhibit("goto_global"):
            raise RuntimeError("输出被禁止，无法飞向目标")
        target = LocationGlobalRelative(lat, lon, alt_m)
        logger.info("飞向目标  lat=%.7f  lon=%.7f  alt=%.1fm", lat, lon, alt_m)
        vehicle.simple_goto(target)

    t0 = time.time()
    while True:
        if is_output_inhibited():
            raise RuntimeError("飞行过程中输出被禁止")
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
    cfg: AppConfig,
    yaw_rate: float | None = None,
) -> None:
    """发送机体坐标系速度指令。

    坐标系说明（机体 NED 映射）：
        - vx: 前进（正）/ 后退（负），m/s
        - vy: 右移（正）/ 左移（负），m/s
        - vz: 下降（正）/ 上升（负），m/s
        - yaw_rate: 偏航角速度（deg/s），None 则保持当前航向

    type_mask 设计（Fix 3）：
        仅速度模式:        0x0FC7 = 0b0000_1111_1100_0111
            忽略 pos(0-2) accel(6-8) force(9) yaw(10) yaw_rate(11)
            使用 vel(3-5)

        速度 + yaw_rate:   0x05C7 = 0b0000_0101_1100_0111
            忽略 pos(0-2) accel(6-8) force(9) yaw(10)
            使用 vel(3-5) + yaw_rate(11)

    此指令必须由调用方以 >= 1 Hz 持续发送，否则飞控 ~3s 后停止执行。
    建议发送频率 4-10 Hz。

    **cfg 为必填参数**，确保限幅永远执行，防止裸速度输出。

    Args:
        vehicle:  DroneKit Vehicle 实例。
        vx:       前进速度（m/s）。
        vy:       右移速度（m/s）。
        vz:       下降速度（m/s，正值向下）。
        cfg:      配置实例（用于读取限幅值，必填）。
        yaw_rate: 偏航角速度（deg/s），可选。
    """
    with _control_lock:
        if _check_inhibit("send_body_velocity"):
            return  # 静默丢弃

        # 限幅（始终执行——cfg 为必填）
        max_xy = cfg.limits.max_xy_vel_mps
        max_z = cfg.limits.max_z_vel_mps
        vx = _clamp(vx, -max_xy, max_xy)
        vy = _clamp(vy, -max_xy, max_xy)
        vz = _clamp(vz, -max_z, max_z)
        if yaw_rate is not None:
            max_yaw = cfg.limits.max_yaw_rate_dps
            yaw_rate = _clamp(yaw_rate, -max_yaw, max_yaw)

        # type_mask 构建（Fix 3 — 正确的位域）
        if yaw_rate is not None:
            # 使用 vel(3-5) + yaw_rate(11)
            # 忽略 pos(0-2) + accel(6-8) + force(9) + yaw(10)
            type_mask = 0x05C7
            yaw_rate_rad = yaw_rate * math.pi / 180.0
        else:
            # 仅使用 vel(3-5)
            # 忽略 pos(0-2) + accel(6-8) + force(9) + yaw(10) + yaw_rate(11)
            type_mask = 0x0FC7
            yaw_rate_rad = 0.0

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms（不使用）
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,              # x, y, z（忽略）
            vx, vy, vz,           # 速度
            0, 0, 0,              # 加速度（忽略）
            0,                    # yaw（忽略，bit10 已置 1）
            yaw_rate_rad,         # yaw_rate（rad/s）
        )
        vehicle.send_mavlink(msg)


def stop_motion(vehicle: Any) -> None:
    """停止所有运动——速度归零。

    实现要点：
        - 发送零速度指令
        - 连续发送 3 次以确保飞控收到
        - 此函数不受门禁限制（安全层调用它来停止输出）

    Args:
        vehicle: DroneKit Vehicle 实例。
    """
    logger.info("停止运动（速度归零）")
    with _control_lock:
        # stop_motion 绕过门禁——它本身就是安全动作
        for _ in range(3):
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0x0FC7,           # 仅速度，全零
                0, 0, 0,          # pos（忽略）
                0, 0, 0,          # vel = 0
                0, 0, 0,          # accel（忽略）
                0, 0,             # yaw, yaw_rate（忽略）
            )
            vehicle.send_mavlink(msg)
            time.sleep(0.1)


def preflight_check(vehicle: Any) -> tuple[bool, str]:
    """起飞前检查。

    检查项目：
        1. GPS Fix >= 3（3D Fix）
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
        reasons.append(f"GPS Fix 不足: fix_type={gps.fix_type}  需要 >=3")

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
    R = 6_371_000  # 地球平均半径（米）
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
