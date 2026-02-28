"""
stream_flask — Flask MJPEG 视频推流

职责：
    通过 HTTP MJPEG 流将摄像头画面推送到浏览器，
    供地面站或调试终端实时查看。

设计原则：
    - 推流读取帧缓存（CameraSource.latest_frame），不直接操作摄像头
    - 推流不能阻塞飞控——Flask 运行在独立线程中
    - Fix 5：帧率从配置读取并传递到 generator
"""

from __future__ import annotations

import logging
import threading
import time
from typing import TYPE_CHECKING, Generator

import cv2
from flask import Flask, Response

if TYPE_CHECKING:
    from src.config_loader import AppConfig
    from src.vision.camera import CameraSource

logger = logging.getLogger(__name__)


def create_app(camera_source: CameraSource, fps: int = 15) -> Flask:
    """创建 Flask MJPEG 推流应用。

    路由：
        ``GET /``         — 简单 HTML 页面（内嵌 <img>）
        ``GET /video_feed`` — MJPEG 流

    Args:
        camera_source: CameraSource 实例（帧来源）。
        fps:           推流帧率（Fix 5：从配置传入）。

    Returns:
        Flask 应用实例。
    """
    app = Flask(__name__)
    app.config["camera_source"] = camera_source
    app.config["stream_fps"] = fps

    @app.route("/")
    def index():
        """简单的推流预览页面。"""
        return (
            '<!DOCTYPE html>'
            '<html><head><title>Drone Camera Stream</title>'
            '<style>body{background:#111;display:flex;justify-content:center;'
            'align-items:center;height:100vh;margin:0;}'
            'img{max-width:100%;border:2px solid #333;}'
            '</style></head>'
            '<body><img src="/video_feed" alt="Camera Stream"></body></html>'
        )

    @app.route("/video_feed")
    def video_feed():
        """MJPEG 视频流端点。"""
        stream_fps = app.config.get("stream_fps", 15)
        return Response(
            _generate_frames(camera_source, fps=stream_fps),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def _generate_frames(
    camera_source: CameraSource,
    fps: int = 15,
) -> Generator[bytes, None, None]:
    """生成 MJPEG 帧流。

    Args:
        camera_source: CameraSource 实例。
        fps:           目标帧率。

    Yields:
        bytes: MJPEG 帧数据。
    """
    interval = 1.0 / max(fps, 1)
    while True:
        frame = camera_source.latest_frame()
        if frame is not None:
            ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ret:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + jpeg.tobytes()
                    + b"\r\n"
                )
        time.sleep(interval)


def run_stream_server(
    camera_source: CameraSource,
    cfg: AppConfig,
) -> None:
    """在独立线程中启动 Flask MJPEG 推流服务器。

    此函数会创建一个 daemon 线程运行 Flask，
    不会阻塞调用方线程。

    Fix 5：将 cfg.vision.stream.fps 传递给 create_app。

    Args:
        camera_source: CameraSource 实例。
        cfg:           全局配置实例。
    """
    host = cfg.vision.stream.host
    port = cfg.vision.stream.port
    fps = cfg.vision.stream.fps

    app = create_app(camera_source, fps=fps)

    def _run():
        logger.info("Flask 推流服务启动  http://%s:%d  fps=%d", host, port, fps)
        app.run(host=host, port=port, threaded=True, use_reloader=False)

    thread = threading.Thread(target=_run, name="FlaskStream", daemon=True)
    thread.start()
    logger.info("推流线程已启动  http://%s:%d", host, port)
