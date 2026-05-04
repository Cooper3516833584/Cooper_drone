"""Takeoff and hover task entrypoint for Cooper_drone."""

from __future__ import annotations

import argparse

from src.app_context import build_app_context
from src.solutions.common import SolutionContext
from src.solutions.takeoff_and_hover import run_takeoff_and_hover


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the takeoff and hover task."""
    parser = argparse.ArgumentParser(description="Run the Cooper_drone takeoff and hover task.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending real MAVLink commands.")
    parser.add_argument("--altitude", type=float, default=1.0, help="Target takeoff altitude in meters.")
    parser.add_argument("--duration", type=float, default=5.0, help="Hover duration in seconds.")
    return parser.parse_args()


def main() -> int:
    """Run the takeoff and hover task."""
    args = parse_args()
    ctx = None
    try:
        ctx = build_app_context(args.config, dry_run=True if args.dry_run else None)
        ctx.mission_was_running = True
        solution_ctx = SolutionContext(ctx.cfg, ctx.movement, ctx.logger, ctx.token, ctx.state_provider)
        run_takeoff_and_hover(solution_ctx, altitude_m=args.altitude, hover_s=args.duration)
        print(f"takeoff hover finished: log_dir={ctx.logger.run_dir}")
        return 0
    except KeyboardInterrupt:
        print("takeoff hover interrupted")
        if ctx is not None:
            ctx.token.cancel("keyboard interrupt")
        return 130
    except Exception as exc:
        print(f"takeoff hover failed: {exc}")
        return 1
    finally:
        if ctx is not None:
            ctx.close()


if __name__ == "__main__":
    raise SystemExit(main())
