from __future__ import annotations

import argparse
import logging
import sys
import time
from enum import Enum

from src import control
from src.config_loader import AppConfig, ConfigError, load_config
from src.fc_link import FlightControllerLink
from src.logx import setup_logging
from src.mav_state import VehicleStateCache
from src.mission_base import CancelToken, MissionContext, build_mission
from src.safety import SafetyManager
from src.telemetry import TelemetryHub


class AppState(str, Enum):
    INIT = "INIT"
    CONNECTING = "CONNECTING"
    STANDBY = "STANDBY"
    RUNNING = "RUNNING"
    SAFE_EXIT = "SAFE_EXIT"
    SHUTDOWN = "SHUTDOWN"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cooper drone controller (pymavlink only)")
    parser.add_argument("--config", required=True, help="YAML 配置路径")
    parser.add_argument("--mission", default="none", help="任务名，默认 none")
    parser.add_argument("--dry-run", action="store_true", help="启用 FakeMavSession")
    parser.add_argument("--log-level", default=None, help="覆盖日志等级")
    parser.add_argument("--standby-seconds", type=float, default=1.0, help="mission=none 时待命秒数")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    state = AppState.INIT

    cfg: AppConfig | None = None
    logger: logging.Logger | None = None
    link: FlightControllerLink | None = None
    telemetry: TelemetryHub | None = None
    safety: SafetyManager | None = None
    cancel = CancelToken()
    selected_mission = (args.mission or "none").strip().lower()
    mission = None
    exit_code = 0

    try:
        cfg = load_config(args.config)
        logger = setup_logging(args.log_level or cfg.app.log_level)
        logger.info("App start dry_run=%s mission=%s", args.dry_run, selected_mission)
        control.bind_runtime_limits(cfg.limits)
        control.clear_motion_inhibit()

        state_cache = VehicleStateCache()
        link = FlightControllerLink(cfg.mavlink, state_cache, dry_run=args.dry_run, logger=logger.getChild("fc_link"))

        state = AppState.CONNECTING
        link.connect()
        session = link.get_session()

        _wait_initial_messages(session, timeout_s=max(3.0, cfg.mavlink.wait_heartbeat_timeout_s))

        telemetry = TelemetryHub(state_cache, cfg.telemetry.sample_hz, logger=logger.getChild("telemetry"))
        telemetry.start()

        safety = SafetyManager(session, cfg, cancel, logger=logger.getChild("safety"))
        safety.start()

        # 统一使用增强 preflight；mission=none 时失败仅告警，不阻塞待命。
        try:
            control.preflight_check(session, cfg)
        except Exception as exc:
            if selected_mission in {"none", "standby"}:
                logger.warning("Preflight warning in standby: %s", exc)
            else:
                raise

        ctx = MissionContext(
            cfg=cfg,
            link=link,
            safety=safety,
            telemetry=telemetry,
            logger=logger.getChild("mission"),
            cancel=cancel,
            vision_bus=None,
        )

        mission = build_mission(selected_mission, ctx)
        if selected_mission in {"none", "standby"}:
            state = AppState.STANDBY
            mission.setup(ctx)
            try:
                mission.run(ctx, cancel)
            finally:
                mission.teardown(ctx)
            if args.standby_seconds > 0:
                time.sleep(args.standby_seconds)
        else:
            state = AppState.RUNNING
            mission.setup(ctx)
            try:
                mission.run(ctx, cancel)
            finally:
                mission.teardown(ctx)

        state = AppState.SHUTDOWN
        logger.info("App finished state=%s", state.value)
        return 0

    except KeyboardInterrupt:
        cancel.cancel("keyboard_interrupt")
        exit_code = 130
        return exit_code
    except (ConfigError, RuntimeError, TimeoutError, ValueError) as exc:
        if logger is not None:
            logger.error("App failed: %s", exc)
        else:
            print(f"ERROR: {exc}", file=sys.stderr)
        exit_code = 1
        return exit_code
    finally:
        state = AppState.SAFE_EXIT
        _safe_exit(
            mission=mission,
            cfg=cfg,
            link=link,
            telemetry=telemetry,
            safety=safety,
            logger=logger,
            run_land=selected_mission not in {"none", "standby"},
        )
        if logger is not None:
            logger.info("Exit state=%s code=%s", state.value, exit_code)


def _wait_initial_messages(session, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = session.state_snapshot()
        hb_ok = state.last_heartbeat_monotonic is not None
        pos_ok = state.position.lat_deg is not None and state.position.lon_deg is not None
        rc_ok = bool(state.rc.channels)
        if hb_ok and pos_ok and rc_ok:
            return
        time.sleep(0.1)
    raise TimeoutError("等待首批关键消息超时（heartbeat/position/rc）")


def _safe_exit(
    *,
    mission,
    cfg: AppConfig | None,
    link: FlightControllerLink | None,
    telemetry: TelemetryHub | None,
    safety: SafetyManager | None,
    logger: logging.Logger | None,
    run_land: bool,
) -> None:
    if telemetry is not None:
        telemetry.stop()
    if safety is not None:
        safety.stop()

    if link is not None and link.is_connected():
        session = link.get_session()
        try:
            control.stop_motion(session)
        except Exception as exc:
            if logger is not None:
                logger.warning("safe_exit stop_motion failed: %s", exc)
        if run_land:
            try:
                control.land_nowait(session)
            except Exception as exc:
                if logger is not None:
                    logger.warning("safe_exit land_nowait failed: %s", exc)
            if cfg is not None:
                try:
                    control.set_mode_nowait(session, cfg.failsafe.revoke_action.value)
                except Exception:
                    pass
    if link is not None:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
