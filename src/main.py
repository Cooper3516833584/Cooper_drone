"""
main — 程序入口：主状态机

职责：
    一个入口把所有模块串起来，形成可运行的骨架。
    即便 mission 未实现，也能启动并做：
    连接/遥测/安全监控/视频推流/等待 RC 允许接管。

主流程状态：
    1. INIT:        加载配置、初始化日志
    2. CONNECT:     连接飞控（支持 dry-run）
    3. PRECHECK:    读取遥测并执行 preflight_check
    4. WAIT_RC:     等待 RC 允许接管（CH5 >= guided_pwm_min）
    5. RUN_MISSION: 选择并执行任务
    6. SAFE_EXIT:   任务结束/取消/异常 -> stop_motion -> land/loiter -> 退出

Fix 1：WAIT_RC 阶段增加 LINK_LOST 检查，
      断链立即进入 SAFE_EXIT，不再无限等待。

命令行参数：
    --config PATH     配置文件路径（默认 config/vehicle.yaml）
    --mission NAME    任务名称（takeoff_and_hover / waypoint_square / vision_track）
    --dry-run         使用 FakeVehicle，不连真机
    --enable-stream   启用 Flask MJPEG 推流
"""

from __future__ import annotations

import argparse
import enum
import logging
import signal
import sys
import time
from typing import Any

logger = logging.getLogger(__name__)


# ── 状态枚举 ──────────────────────────────────────────────


class State(enum.Enum):
    INIT = "INIT"
    CONNECT = "CONNECT"
    PRECHECK = "PRECHECK"
    WAIT_RC = "WAIT_RC"
    RUN_MISSION = "RUN_MISSION"
    SAFE_EXIT = "SAFE_EXIT"


# ── 参数解析 ──────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="无人机上位机控制框架 (ArduCopter + MAVLink)",
    )
    parser.add_argument(
        "--config", type=str, default="config/vehicle.yaml",
        help="配置文件路径（默认: config/vehicle.yaml）",
    )
    parser.add_argument(
        "--mission", type=str, default=None,
        help="任务名称: takeoff_and_hover / waypoint_square / vision_track",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="使用 FakeVehicle 模拟，不连真机",
    )
    parser.add_argument(
        "--enable-stream", action="store_true",
        help="启用 Flask MJPEG 视频推流",
    )
    return parser.parse_args()


# ── 主状态机 ──────────────────────────────────────────────


class DroneStateMachine:
    """主状态机——串联所有子系统。

    Usage::

        sm = DroneStateMachine(args)
        sm.run()
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._state = State.INIT

        # 子系统引用（在 INIT 状态初始化）
        self._cfg: Any = None
        self._link: Any = None
        self._safety: Any = None
        self._telemetry: Any = None
        self._camera: Any = None
        self._tracker: Any = None
        self._shutdown_requested = False

    def run(self) -> None:
        """运行状态机主循环。"""
        # 注册 SIGINT / SIGTERM 处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        while not self._shutdown_requested:
            try:
                logger.info("== 状态: %s ==", self._state.value)
                next_state = self._dispatch()
                if next_state is None:
                    break
                self._state = next_state
            except KeyboardInterrupt:
                logger.info("收到键盘中断")
                self._state = State.SAFE_EXIT
            except Exception as exc:
                logger.error("状态机异常: %s", exc, exc_info=True)
                self._state = State.SAFE_EXIT

        logger.info("程序退出")

    def _dispatch(self) -> State | None:
        """根据当前状态分发到对应处理函数。"""
        handlers: dict[State, Any] = {
            State.INIT: self._state_init,
            State.CONNECT: self._state_connect,
            State.PRECHECK: self._state_precheck,
            State.WAIT_RC: self._state_wait_rc,
            State.RUN_MISSION: self._state_run_mission,
            State.SAFE_EXIT: self._state_safe_exit,
        }
        handler = handlers.get(self._state)
        if handler is None:
            logger.error("未知状态: %s", self._state)
            return None
        return handler()

    # ── 1. INIT ──────────────────────────────

    def _state_init(self) -> State:
        """加载配置、初始化日志。"""
        from src.config_loader import load_config
        from src.logx import init_logging

        self._cfg = load_config(self._args.config)
        init_logging(self._cfg)

        logger.info("============================================")
        logger.info("  Drone Mission Control Framework v0.1.0")
        logger.info("  dry-run=%s  stream=%s  mission=%s",
                     self._args.dry_run, self._args.enable_stream,
                     self._args.mission or "(none)")
        logger.info("============================================")

        return State.CONNECT

    # ── 2. CONNECT ───────────────────────────

    def _state_connect(self) -> State:
        """连接飞控。"""
        from src.fc_link import FlightControllerLink

        self._link = FlightControllerLink(self._cfg, dry_run=self._args.dry_run)

        try:
            self._link.connect()
        except ConnectionError as exc:
            logger.error("飞控连接失败: %s", exc)
            return State.SAFE_EXIT

        return State.PRECHECK

    # ── 3. PRECHECK ──────────────────────────

    def _state_precheck(self) -> State:
        """启动遥测、安全监控、（可选）视频推流，并执行起飞前检查。"""
        from src.safety import SafetyManager
        from src.telemetry import TelemetryHub

        # 遥测
        self._telemetry = TelemetryHub(self._cfg, self._link)
        self._telemetry.start()

        # 安全监控
        self._safety = SafetyManager(self._cfg, self._link)
        self._safety.start()

        # 视频推流（CLI --enable-stream 或配置 vision.stream.enabled）
        if self._args.enable_stream or self._cfg.vision.stream.enabled:
            self._start_vision()

        # 等待遥测稳定
        time.sleep(1.0)

        # 起飞前检查
        from src.control import preflight_check
        vehicle = self._link.get_vehicle()
        passed, reason = preflight_check(vehicle)

        if not passed:
            logger.warning("起飞前检查未通过: %s", reason)
            # 不退出——仍然进入 WAIT_RC，让飞手决策

        snap = self._telemetry.latest()
        logger.info(
            "遥测快照  mode=%s  armed=%s  alt=%.1fm  GPS=%d(%dsat)  EKF=%s  batt=%.1fV",
            snap.mode, snap.armed, snap.alt_rel_m,
            snap.gps_fix, snap.gps_num_sat, snap.ekf_ok,
            snap.battery_v,
        )

        return State.WAIT_RC

    # ── 4. WAIT_RC ───────────────────────────

    def _state_wait_rc(self) -> State:
        """等待 RC 允许上位机接管（CH5 >= guided_pwm_min）。

        Fix 1：增加 LINK_LOST 检查——断链立即进入 SAFE_EXIT，
        不再无限卡住在此循环。
        """
        from src.safety import SafetyState

        if self._args.mission is None:
            logger.info("未指定任务，系统进入待机状态（Ctrl+C 退出）")
            self._standby_loop()
            return State.SAFE_EXIT

        logger.info("等待 RC 允许接管...(CH%d >= %d)",
                     self._cfg.rc.mode_channel, self._cfg.rc.guided_pwm_min)

        while not self._shutdown_requested:
            safety_state = self._safety.get_safety_state()

            # Fix 1: 检查紧急停桨
            if safety_state == SafetyState.KILL:
                logger.error("等待期间触发紧急停桨")
                return State.SAFE_EXIT

            # Fix 1: 检查断链——不再无限等待
            if safety_state == SafetyState.LINK_LOST:
                logger.error("等待期间检测到链路丢失，进入安全退出")
                return State.SAFE_EXIT

            # 允许接管
            if safety_state == SafetyState.GUIDED_ALLOWED:
                logger.info("RC 允许接管，进入任务执行")
                return State.RUN_MISSION

            # dry-run 自动允许
            if self._args.dry_run:
                logger.info("[DRY-RUN] 自动允许接管")
                return State.RUN_MISSION

            time.sleep(0.2)

        return State.SAFE_EXIT

    # ── 5. RUN_MISSION ───────────────────────

    def _state_run_mission(self) -> State:
        """选择并执行任务。"""
        from src.mission_base import CancelToken, MissionContext
        from src.missions import MISSION_REGISTRY

        mission_name = self._args.mission
        if mission_name not in MISSION_REGISTRY:
            logger.error("未知任务: %s  可选: %s",
                         mission_name, list(MISSION_REGISTRY.keys()))
            return State.SAFE_EXIT

        # 创建任务实例
        task_cls = MISSION_REGISTRY[mission_name]
        task = task_cls()

        # 创建 cancel token 并注册到安全层
        token = CancelToken()
        self._safety.register_cancel_token(token)

        # 创建任务上下文
        ctx = MissionContext(
            cfg=self._cfg,
            link=self._link,
            safety=self._safety,
            telemetry=self._telemetry,
            vision_bus=self._tracker,
        )

        # 执行任务
        logger.info("开始执行任务: %s", mission_name)
        try:
            task.execute(ctx, token)
        except NotImplementedError:
            logger.warning("任务 '%s' 尚未实现（占位）", mission_name)
        except Exception as exc:
            logger.error("任务执行异常: %s", exc, exc_info=True)
        finally:
            self._safety.unregister_cancel_token(token)

        return State.SAFE_EXIT

    # ── 6. SAFE_EXIT ─────────────────────────

    def _state_safe_exit(self) -> State | None:
        """安全退出——停止控制输出、降落/悬停、关闭所有子系统。"""
        logger.info("执行安全退出流程...")

        # 停止运动 + 按策略执行 LAND/LOITER
        if self._link and self._link.is_connected():
            try:
                from src.control import stop_motion
                vehicle = self._link.get_vehicle()
                stop_motion(vehicle)
            except Exception as exc:
                logger.warning("stop_motion 失败: %s", exc)

            # Fix G: 如果飞机已 armed，按 failsafe 策略执行 LAND/LOITER
            try:
                vehicle = self._link.get_vehicle()
                if vehicle.armed:
                    action = self._cfg.failsafe.link_lost_action.upper()
                    if action == "LOITER":
                        from src.control import set_mode_nowait
                        logger.info("SAFE_EXIT: 切换到 LOITER")
                        set_mode_nowait(vehicle, "LOITER")
                    else:
                        from src.control import land
                        logger.info("SAFE_EXIT: 执行 LAND")
                        land(vehicle, timeout_s=30.0)
            except Exception as exc:
                logger.warning("SAFE_EXIT land/loiter 失败: %s", exc)

        # 停止子系统
        if self._tracker:
            try:
                self._tracker.stop()
            except Exception:
                pass
        if self._camera:
            try:
                self._camera.stop()
            except Exception:
                pass
        if self._safety:
            try:
                self._safety.stop()
            except Exception:
                pass
        if self._telemetry:
            try:
                self._telemetry.stop()
            except Exception:
                pass
        if self._link:
            try:
                self._link.disconnect()
            except Exception:
                pass

        logger.info("所有子系统已停止，程序退出")
        return None  # 结束状态机

    # ── 辅助方法 ──────────────────────────────

    def _start_vision(self) -> None:
        """启动视觉子系统（摄像头 + 跟踪器 + 推流）。"""
        try:
            from src.vision.camera import CameraSource
            from src.vision.stream_flask import run_stream_server
            from src.vision.tracker import Tracker

            self._camera = CameraSource(self._cfg)
            self._camera.start()

            self._tracker = Tracker(self._camera)
            self._tracker.start()

            run_stream_server(self._camera, self._cfg)
        except Exception as exc:
            logger.warning("视觉子系统启动失败: %s（继续运行）", exc)
            self._camera = None
            self._tracker = None

    def _standby_loop(self) -> None:
        """待机循环——定期输出遥测信息。

        Fix 1 同样适用：待机循环也检查 LINK_LOST。
        """
        from src.safety import SafetyState

        logger.info("进入待机模式  按 Ctrl+C 退出")
        while not self._shutdown_requested:
            try:
                safety_state = self._safety.get_safety_state()
                if safety_state in (SafetyState.KILL, SafetyState.LINK_LOST):
                    logger.warning("待机期间检测到安全事件 (%s)，退出待机",
                                    safety_state.value)
                    return

                snap = self._telemetry.latest()
                logger.info(
                    "待机  mode=%s  armed=%s  alt=%.1fm  safety=%s",
                    snap.mode, snap.armed, snap.alt_rel_m,
                    safety_state.value,
                )
                time.sleep(2.0)
            except Exception:
                time.sleep(1.0)

    def _signal_handler(self, signum: int, frame: Any) -> None:
        """信号处理——优雅退出。"""
        logger.info("收到信号 %d，准备退出...", signum)
        self._shutdown_requested = True


# ── 入口 ──────────────────────────────────────────────────


def main() -> None:
    """程序入口。"""
    args = parse_args()
    sm = DroneStateMachine(args)
    sm.run()


if __name__ == "__main__":
    main()
