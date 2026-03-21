from __future__ import annotations

import logging
import threading
from typing import Any

from flask import Flask, jsonify


class VisionStreamServer:
    """可选 Flask 流服务占位。默认不启用，且不阻塞主控制流程。"""

    def __init__(self, *, host: str = "0.0.0.0", port: int = 5000, logger: logging.Logger | None = None) -> None:
        self._host = host
        self._port = port
        self._log = logger or logging.getLogger(__name__)
        self._thread: threading.Thread | None = None
        self._app = Flask("vision_stream")
        self._build_routes()

    def _build_routes(self) -> None:
        @self._app.get("/health")
        def health() -> Any:
            return jsonify({"ok": True, "service": "vision_stream_placeholder"})

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        def _run() -> None:
            self._log.info("Starting vision stream placeholder on %s:%s", self._host, self._port)
            self._app.run(host=self._host, port=self._port, debug=False, use_reloader=False)

        self._thread = threading.Thread(target=_run, name="vision-stream", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        # Flask 内置服务器不支持优雅 stop，本占位实现保持 no-op。
        return

