"""
logx — 日志初始化

职责：
    为整个项目提供统一的日志配置：控制台输出 + 文件滚动。
    真机调参靠日志复盘，必须开箱即用。

日志格式：
    时间 | 级别 | 模块名 | 消息

用法：
    from src.logx import init_logging
    init_logging(cfg)

    import logging
    logger = logging.getLogger(__name__)
    logger.info("起飞完成")
"""

from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.config_loader import AppConfig


_LOG_FORMAT = "%(asctime)s | %(levelname)-7s | %(name)s | %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def init_logging(cfg: AppConfig) -> None:
    """初始化全局日志系统。

    配置内容：
        1. 控制台 Handler（StreamHandler → stderr）
        2. 文件 Handler（RotatingFileHandler，按大小滚动）
        3. 日志级别、格式由 ``cfg.logging`` 控制

    Args:
        cfg: 全局配置实例（AppConfig）。

    Side Effects:
        - 创建日志目录（如不存在）
        - 配置 root logger
    """
    log_dir = Path(cfg.logging.dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    level = getattr(logging, cfg.logging.level.upper(), logging.INFO)
    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_DATE_FORMAT)

    # ── Root Logger ──────────────────────────
    root = logging.getLogger()
    root.setLevel(level)
    # 清除已有 handler（避免重复初始化叠加）
    root.handlers.clear()

    # ── 控制台 ────────────────────────────────
    console = logging.StreamHandler(sys.stderr)
    console.setLevel(level)
    console.setFormatter(formatter)
    root.addHandler(console)

    # ── 文件（滚动） ─────────────────────────
    log_file = log_dir / "drone_mission.log"
    file_handler = RotatingFileHandler(
        filename=str(log_file),
        maxBytes=cfg.logging.max_bytes,
        backupCount=cfg.logging.backup_count,
        encoding="utf-8",
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)
    root.addHandler(file_handler)

    # 降低第三方库噪音
    for noisy in ("dronekit", "pymavlink", "MAVLink", "urllib3", "werkzeug"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    root.info("日志系统已初始化  level=%s  dir=%s", cfg.logging.level, log_dir.resolve())
