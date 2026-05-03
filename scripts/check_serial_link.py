"""Read-only MAVLink serial link check."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config_loader import load_config
from src.logging_runtime import RuntimeLogger
from src.mavlink_connection import MavlinkConnection


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Check a MAVLink serial or UDP link without sending commands.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Skip real MAVLink connection.")
    return parser.parse_args()


def main() -> int:
    """Run a read-only MAVLink link check."""
    args = parse_args()
    cfg = load_config(args.config, dry_run_override=True if args.dry_run else None)

    if cfg.dry_run:
        print("dry-run: skipping MAVLink serial link check")
        print(f"profile: {cfg.profile_name}")
        print(f"connection_string: {cfg.mavlink.connection_string}")
        return 0

    logger = RuntimeLogger(
        run_id="serial-link-check",
        run_dir=Path("."),
        app_logger=logging.getLogger("check_serial_link"),
    )
    connection = MavlinkConnection(cfg.mavlink, logger)
    try:
        connection.connect()
        state = connection.wait_heartbeat()
        print(f"target_system: {cfg.mavlink.target_system}")
        print(f"target_component: {cfg.mavlink.target_component}")
        print(f"mode: {state.mode}")
        print(f"armed: {state.armed}")
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
