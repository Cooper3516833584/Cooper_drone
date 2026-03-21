from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraConfig:
    index: int = 0
    width: int = 640
    height: int = 480


class Camera:
    """摄像头占位实现。本轮不接入真实视觉算法。"""

    def __init__(self, cfg: CameraConfig | None = None) -> None:
        self._cfg = cfg or CameraConfig()
        self._opened = False

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def is_opened(self) -> bool:
        return self._opened

    def read(self) -> tuple[bool, Any]:
        if not self._opened:
            return False, None
        # 占位返回，不阻塞主控制流程。
        return True, None

