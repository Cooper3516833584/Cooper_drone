from __future__ import annotations

from src.mission_base import CancelToken, MissionContext, MissionTask


class WaypointSquareMission(MissionTask):
    """占位任务：后续应通过 control.goto_global + telemetry 判定到点。"""

    name = "waypoint_square"

    def setup(self, ctx: MissionContext) -> None:
        ctx.logger.info("setup waypoint_square placeholder")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        del ctx, token
        raise NotImplementedError("waypoint_square not implemented yet")

    def teardown(self, ctx: MissionContext) -> None:
        ctx.logger.info("teardown waypoint_square placeholder")

