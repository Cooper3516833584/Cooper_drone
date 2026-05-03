"""Vision follow task entrypoint for Cooper_drone."""

from __future__ import annotations

import argparse

from src.Vision.camera import DummyCameraSource, OpenCVCameraSource
from src.Vision.detector import DummyDetector
from src.Vision.tracker import CenteringTracker
from src.app_context import build_app_context
from src.solutions.common import SolutionContext
from src.solutions.vision_follow import run_vision_follow


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the vision follow task."""
    parser = argparse.ArgumentParser(description="Run the Cooper_drone vision follow task.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending real MAVLink commands.")
    parser.add_argument("--altitude", type=float, default=1.0, help="Target takeoff altitude in meters.")
    parser.add_argument("--duration", type=float, default=5.0, help="Vision follow duration in seconds.")
    return parser.parse_args()


def main() -> int:
    """Run the vision follow task."""
    args = parse_args()
    ctx = None
    try:
        ctx = build_app_context(args.config, dry_run=True if args.dry_run else None)
        camera = _build_camera(ctx)
        detector = DummyDetector()
        tracker = CenteringTracker()
        ctx.logger.event(
            "vision_modules_created",
            camera=camera.__class__.__name__,
            detector=detector.__class__.__name__,
            tracker=tracker.__class__.__name__,
        )
        solution_ctx = SolutionContext(ctx.cfg, ctx.movement, ctx.logger, ctx.token, ctx.state_provider)
        run_vision_follow(solution_ctx, altitude_m=args.altitude, duration_s=args.duration)
        print(f"vision follow finished: log_dir={ctx.logger.run_dir}")
        return 0
    except Exception as exc:
        print(f"vision follow failed: {exc}")
        return 1
    finally:
        if ctx is not None:
            ctx.close()


def _build_camera(ctx):
    if ctx.cfg.dry_run or not ctx.cfg.vision.enabled:
        return DummyCameraSource(
            width=ctx.cfg.vision.width,
            height=ctx.cfg.vision.height,
            fps=ctx.cfg.vision.fps,
        )
    return OpenCVCameraSource(
        camera_index=ctx.cfg.vision.camera_index,
        width=ctx.cfg.vision.width,
        height=ctx.cfg.vision.height,
        fps=ctx.cfg.vision.fps,
    )


if __name__ == "__main__":
    raise SystemExit(main())
