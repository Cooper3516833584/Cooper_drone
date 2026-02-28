"""
vision_track — 视觉跟踪飞行任务（占位）

未来实现说明：
    1. setup:  确认 GUIDED + armed + CameraSource 运行 + Tracker 运行
    2. run:    循环读取 Tracker.latest_target()
               如果 target.detected 且 not is_stale():
                   将 dx/dy 映射为 vx/vy（P 或 PID 控制器）
                   调用 control.send_body_velocity(vehicle, vx, vy, 0)
               否则：
                   control.stop_motion(vehicle)（或低速悬停）
               每次循环检查 token.is_cancelled()
    3. teardown: control.stop_motion(vehicle)

使用的控制原语：
    - control.set_mode("GUIDED")
    - control.send_body_velocity(vehicle, vx, vy, vz, yaw_rate)
    - control.stop_motion(vehicle)
    - control.land(vehicle)

使用的视觉接口：
    - ctx.vision_bus.latest_target() -> VisionTarget
"""

from __future__ import annotations

from src.mission_base import CancelToken, MissionContext, MissionTask


class VisionTrackTask(MissionTask):
    """视觉跟踪——利用摄像头检测目标并用速度指令跟随。

    Args:
        timeout_s: 任务超时（秒），默认 120s。
    """

    def __init__(self, timeout_s: float = 120.0) -> None:
        super().__init__(name="VisionTrack", timeout_s=timeout_s)

    def setup(self, ctx: MissionContext) -> None:
        """任务初始化。

        TODO:
            - 确认 GUIDED + armed
            - 确认 CameraSource + Tracker 正常运行
            - 可选：初始化 PID 控制器
        """
        raise NotImplementedError("VisionTrackTask.setup() 尚未实现")

    def run(self, ctx: MissionContext, token: CancelToken) -> None:
        """任务主逻辑。

        TODO:
            - 控制循环 (~10-20 Hz):
                target = ctx.vision_bus.latest_target()
                if target.detected and not target.is_stale():
                    vx, vy = pid(target.dx, target.dy)
                    send_body_velocity(vehicle, vx, vy, 0)
                else:
                    stop_motion(vehicle)
                if token.is_cancelled(): break
                sleep(0.05)
        """
        raise NotImplementedError("VisionTrackTask.run() 尚未实现")

    def teardown(self, ctx: MissionContext) -> None:
        """任务清理。

        TODO:
            - stop_motion(vehicle)
        """
        raise NotImplementedError("VisionTrackTask.teardown() 尚未实现")
