from __future__ import annotations

import time

from src import control
from src.mission_base import CancelToken, MissionContext, MissionTask


class TakeoffAndHoverMission(MissionTask):
    name = "takeoff_and_hover"

    def setup(self, ctx: MissionContext) -> None:
        # 轻量检查，复杂校验放在控制层 preflight_check。
        if not ctx.link.is_connected():
            raise RuntimeError("link 未连接，不能执行 takeoff_and_hover")
        ctx.logger.info("setup takeoff_and_hover")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        token.raise_if_cancelled()
        target_alt = ctx.cfg.limits.takeoff_alt_m
        hover_s = ctx.cfg.limits.hover_seconds

        ctx.logger.info("run takeoff_and_hover target_alt=%.2f hover=%.2fs", target_alt, hover_s)
        control.takeoff(ctx.session, alt_m=target_alt, timeout_s=30.0)

        hover_deadline = time.monotonic() + hover_s
        while time.monotonic() < hover_deadline:
            if token.is_cancelled():
                ctx.logger.warning("mission cancelled during hover: %s", token.reason)
                break
            time.sleep(0.1)

        token.raise_if_cancelled()
        control.land(ctx.session, timeout_s=45.0)

    def teardown(self, ctx: MissionContext) -> None:
        control.stop_motion(ctx.session)
        ctx.logger.info("teardown takeoff_and_hover")
