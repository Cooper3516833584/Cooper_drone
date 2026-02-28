"""
takeoff_and_hover — 起飞并悬停任务（占位）

未来实现说明：
    1. setup:  切换 GUIDED → arm → preflight_check
    2. run:    调用 control.takeoff(alt_m) → 悬停指定时间 → 循环检查 token
    3. teardown: control.stop_motion() → control.land()

使用的控制原语：
    - control.set_mode("GUIDED")
    - control.arm(vehicle)
    - control.takeoff(vehicle, alt_m, timeout_s)
    - control.stop_motion(vehicle)
    - control.land(vehicle)
"""

from __future__ import annotations

from src.mission_base import CancelToken, MissionContext, MissionTask


class TakeoffAndHoverTask(MissionTask):
    """起飞至指定高度并悬停。

    Args:
        hover_alt_m:  悬停高度（米），默认从配置读取。
        hover_time_s: 悬停时间（秒），默认 10s。
        timeout_s:    任务超时（秒），默认 60s。
    """

    def __init__(
        self,
        hover_alt_m: float | None = None,
        hover_time_s: float = 10.0,
        timeout_s: float = 60.0,
    ) -> None:
        super().__init__(name="TakeoffAndHover", timeout_s=timeout_s)
        self._hover_alt_m = hover_alt_m
        self._hover_time_s = hover_time_s

    def setup(self, ctx: MissionContext) -> None:
        """任务初始化。

        TODO:
            - 读取配置中的 takeoff_alt_m（若未显式指定）
            - preflight_check
            - set_mode("GUIDED")
            - arm
        """
        raise NotImplementedError("TakeoffAndHoverTask.setup() 尚未实现")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        """任务主逻辑。

        TODO:
            - takeoff(vehicle, alt_m)
            - 悬停循环：每 0.5s 检查 token
            - 悬停时间到期后返回
        """
        raise NotImplementedError("TakeoffAndHoverTask.run() 尚未实现")

    def teardown(self, ctx: MissionContext) -> None:
        """任务清理。

        TODO:
            - stop_motion(vehicle)
            - land(vehicle)
        """
        raise NotImplementedError("TakeoffAndHoverTask.teardown() 尚未实现")
