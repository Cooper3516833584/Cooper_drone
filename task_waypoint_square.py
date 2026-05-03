"""Waypoint square task entrypoint for Cooper_drone."""

from __future__ import annotations

import argparse

from src.app_context import build_app_context
from src.solutions.common import SolutionContext
from src.solutions.waypoint_square import run_waypoint_square


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the waypoint square task."""
    parser = argparse.ArgumentParser(description="Run the Cooper_drone waypoint square task.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending real MAVLink commands.")
    parser.add_argument("--altitude", type=float, default=1.0, help="Target takeoff altitude in meters.")
    parser.add_argument("--side", type=float, default=1.0, help="Square side length in meters.")
    parser.add_argument("--speed", type=float, default=0.5, help="Body velocity speed in meters per second.")
    parser.add_argument("--duration", type=float, default=0.0, help="Unused compatibility option.")
    return parser.parse_args()


def main() -> int:
    """Run the waypoint square task."""
    args = parse_args()
    ctx = None
    try:
        ctx = build_app_context(args.config, dry_run=True if args.dry_run else None)
        solution_ctx = SolutionContext(ctx.cfg, ctx.movement, ctx.logger, ctx.token, ctx.state_provider)
        run_waypoint_square(
            solution_ctx,
            altitude_m=args.altitude,
            side_m=args.side,
            speed_mps=args.speed,
        )
        print(f"waypoint square finished: log_dir={ctx.logger.run_dir}")
        return 0
    except Exception as exc:
        print(f"waypoint square failed: {exc}")
        return 1
    finally:
        if ctx is not None:
            ctx.close()


if __name__ == "__main__":
    raise SystemExit(main())
