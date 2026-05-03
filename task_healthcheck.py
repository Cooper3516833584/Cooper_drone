"""Healthcheck task entrypoint for Cooper_drone."""

from __future__ import annotations

import argparse

from src.app_context import build_app_context


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the healthcheck task."""
    parser = argparse.ArgumentParser(description="Run the Cooper_drone healthcheck task.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Run without sending real MAVLink commands.")
    parser.add_argument("--altitude", type=float, default=1.0, help="Unused compatibility option.")
    parser.add_argument("--duration", type=float, default=0.0, help="Unused compatibility option.")
    return parser.parse_args()


def main() -> int:
    """Run the healthcheck task."""
    args = parse_args()
    ctx = None
    try:
        ctx = build_app_context(args.config, dry_run=True if args.dry_run else None)
        ctx.logger.event("healthcheck_started", config=args.config, dry_run=ctx.cfg.dry_run)
        print(f"profile: {ctx.cfg.profile_name}")
        print(f"dry_run: {ctx.cfg.dry_run}")
        print(f"Log directory: {ctx.logger.run_dir}")
        if ctx.cfg.dry_run:
            print("dry-run: skipping MAVLink connection")
        else:
            state = ctx.connection.state_snapshot() if ctx.connection is not None else None
            print(
                "heartbeat: "
                f"last_heartbeat_ts={state.last_heartbeat_ts}, mode={state.mode}, armed={state.armed}"
            )
        ctx.logger.event("healthcheck_finished", status="ok")
        return 0
    except Exception as exc:
        print(f"healthcheck failed: {exc}")
        return 1
    finally:
        if ctx is not None:
            ctx.close()


if __name__ == "__main__":
    raise SystemExit(main())
