"""
waypoint_square — 正方形航点飞行任务（占位）

未来实现说明：
    1. setup:  确认 GUIDED 模式 + armed + 当前位置作为起点
    2. run:    以当前位置为中心，依次飞往 4 个正方形顶点
               每段飞行调用 control.goto_global()
               每段之间检查 token.is_cancelled()
    3. teardown: control.stop_motion() → 回到起点 → 可选 land

使用的控制原语：
    - control.set_mode("GUIDED")
    - control.goto_global(vehicle, lat, lon, alt_m, timeout_s)
    - control.stop_motion(vehicle)
    - control.land(vehicle)
"""

from __future__ import annotations

from src.mission_base import CancelToken, MissionContext, MissionTask


class WaypointSquareTask(MissionTask):
    """正方形航点飞行——绕正方形路径飞行一圈。

    Args:
        side_m:    正方形边长（米），默认 5m。
        alt_m:     飞行高度（米），默认从配置读取。
        timeout_s: 任务超时（秒），默认 120s。
    """

    def __init__(
        self,
        side_m: float = 5.0,
        alt_m: float | None = None,
        timeout_s: float = 120.0,
    ) -> None:
        super().__init__(name="WaypointSquare", timeout_s=timeout_s)
        self._side_m = side_m
        self._alt_m = alt_m

    def setup(self, ctx: MissionContext) -> None:
        """任务初始化。

        TODO:
            - 确认 GUIDED + armed
            - 记录当前 GPS 位置作为起点
            - 计算 4 个正方形顶点 GPS 坐标
        """
        raise NotImplementedError("WaypointSquareTask.setup() 尚未实现")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        """任务主逻辑。

        TODO:
            - for waypoint in [NE, SE, SW, NW]:
                if token.is_cancelled(): break
                goto_global(vehicle, wp.lat, wp.lon, alt_m)
            - 回到起点
        """
        raise NotImplementedError("WaypointSquareTask.run() 尚未实现")

    def teardown(self, ctx: MissionContext) -> None:
        """任务清理。

        TODO:
            - stop_motion(vehicle)
            - land(vehicle) 或 返回起点
        """
        raise NotImplementedError("WaypointSquareTask.teardown() 尚未实现")
