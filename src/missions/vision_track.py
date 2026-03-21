from __future__ import annotations

from src.mission_base import CancelToken, MissionContext, MissionTask


class VisionTrackMission(MissionTask):
    """占位任务：后续应周期调用 control.send_body_velocity 做视觉闭环。"""

    name = "vision_track"

    def setup(self, ctx: MissionContext) -> None:
        ctx.logger.info("setup vision_track placeholder")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        del ctx, token
        raise NotImplementedError("vision_track not implemented yet")

    def teardown(self, ctx: MissionContext) -> None:
        ctx.logger.info("teardown vision_track placeholder")

