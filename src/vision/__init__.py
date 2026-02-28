# drone_mission.src.vision 包初始化
"""
视觉子系统包。

设计原则：
    - 视觉与飞控解耦——Tracker 只输出偏差，不直接操作飞机
    - 推流不阻塞控制——Flask 推流与 Tracker 各自独立线程
    - VisionTarget 必须带 timestamp，过期判定 is_stale
"""
