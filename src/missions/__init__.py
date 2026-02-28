# drone_mission.src.missions 包初始化
"""
飞行任务包。

所有任务必须继承 ``MissionTask`` 并实现 ``setup / run / teardown``。
当前为占位实现，后续逐步填充控制逻辑。
"""

from src.missions.takeoff_and_hover import TakeoffAndHoverTask
from src.missions.waypoint_square import WaypointSquareTask
from src.missions.vision_track import VisionTrackTask

# 任务注册表——main.py 通过名称查找任务类
MISSION_REGISTRY: dict[str, type] = {
    "takeoff_and_hover": TakeoffAndHoverTask,
    "waypoint_square": WaypointSquareTask,
    "vision_track": VisionTrackTask,
}

__all__ = [
    "MISSION_REGISTRY",
    "TakeoffAndHoverTask",
    "WaypointSquareTask",
    "VisionTrackTask",
]
