"""Print the resolved Cooper_drone configuration."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import yaml

from src.config_loader import dump_resolved_config, load_config


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Print the resolved Cooper_drone configuration.")
    parser.add_argument("--config", required=True, help="Path to the YAML configuration file.")
    parser.add_argument("--dry-run", action="store_true", help="Force dry-run mode while resolving config.")
    return parser.parse_args()


def main() -> int:
    """Print resolved configuration as YAML."""
    args = parse_args()
    cfg = load_config(args.config, dry_run_override=True if args.dry_run else None)
    print(yaml.safe_dump(dump_resolved_config(cfg), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
