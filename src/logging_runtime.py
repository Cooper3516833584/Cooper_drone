"""Runtime logging support for Cooper_drone task runs."""

from __future__ import annotations

import json
import logging
import re
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO
from uuid import uuid4

import yaml

from src.config_loader import AppConfig, dump_resolved_config


JSONL_FILENAMES = {
    "event": "events.jsonl",
    "command": "commands.jsonl",
    "safety": "safety.jsonl",
    "telemetry": "telemetry.jsonl",
}


@dataclass
class RuntimeLogger:
    """Runtime logger for one task execution."""

    run_id: str
    run_dir: Path
    app_logger: logging.Logger
    _jsonl_files: dict[str, TextIO] = field(default_factory=dict, repr=False)
    _handlers: list[logging.Handler] = field(default_factory=list, repr=False)
    _closed: bool = field(default=False, repr=False)

    def event(self, name: str, **fields: Any) -> None:
        """Write an event record."""
        self._write_jsonl("event", name, fields)

    def command(self, name: str, **fields: Any) -> None:
        """Write a command record."""
        self._write_jsonl("command", name, fields)

    def safety(self, name: str, **fields: Any) -> None:
        """Write a safety record."""
        self._write_jsonl("safety", name, fields)

    def telemetry(self, **fields: Any) -> None:
        """Write a telemetry record."""
        self._write_jsonl("telemetry", "telemetry", fields)

    def close(self) -> None:
        """Flush and close all runtime log files."""
        if self._closed:
            return

        self.app_logger.info("Runtime logging closed at %s", datetime.now().isoformat(timespec="seconds"))

        for handle in self._jsonl_files.values():
            handle.flush()
            handle.close()

        for handler in self._handlers:
            handler.flush()
            handler.close()
            self.app_logger.removeHandler(handler)

        self._closed = True

    def _write_jsonl(self, record_type: str, name: str, fields: dict[str, Any]) -> None:
        if self._closed:
            return

        record = {
            "ts": time.time(),
            "run_id": self.run_id,
            "type": record_type,
            "name": name,
        }
        record.update(fields)

        try:
            handle = self._jsonl_files[record_type]
            handle.write(json.dumps(record, ensure_ascii=True, sort_keys=True) + "\n")
            handle.flush()
        except Exception:
            self.app_logger.exception("Failed to write %s JSONL record", record_type)


def setup_runtime_logging(cfg: AppConfig) -> RuntimeLogger:
    """Create a runtime log directory and initialize text and JSONL logging."""
    run_id = _make_run_id(cfg)
    run_dir = Path(cfg.logging.log_dir) / run_id
    run_dir.mkdir(parents=True, exist_ok=False)

    app_log_path = run_dir / "app.log"
    logger = logging.getLogger(f"cooper_drone.runtime.{run_id}")
    logger.setLevel(_level_from_name(cfg.logging.console_level))
    logger.propagate = False

    file_handler = logging.FileHandler(app_log_path, encoding="utf-8")
    file_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(file_handler)

    jsonl_files = {
        record_type: (run_dir / filename).open("a", encoding="utf-8")
        for record_type, filename in JSONL_FILENAMES.items()
    }

    with (run_dir / "resolved_config.yaml").open("w", encoding="utf-8") as handle:
        yaml.safe_dump(dump_resolved_config(cfg), handle, sort_keys=True)

    runtime_logger = RuntimeLogger(
        run_id=run_id,
        run_dir=run_dir,
        app_logger=logger,
        _jsonl_files=jsonl_files,
        _handlers=[file_handler],
    )

    logger.info("Runtime logging started at %s", datetime.now().isoformat(timespec="seconds"))
    logger.info("run_id=%s", run_id)
    logger.info("config profile=%s", cfg.profile_name)
    logger.info("dry_run=%s", cfg.dry_run)
    return runtime_logger


def _make_run_id(cfg: AppConfig) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    profile = _safe_run_id_part(cfg.profile_name)
    suffix = uuid4().hex[:5]
    return f"{timestamp}_{profile}_{suffix}"


def _safe_run_id_part(value: str) -> str:
    safe = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip())
    return safe or "run"


def _level_from_name(level_name: str) -> int:
    level = logging.getLevelName(level_name.upper())
    if isinstance(level, int):
        return level
    return logging.INFO
