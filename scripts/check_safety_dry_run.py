"""Simulate safety events in dry-run mode — no MAVLink connection needed.

Prints the expected safety decision, events, and actions for each scenario
so the operator can confirm the safety policy is configured correctly
before connecting to real hardware.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config_loader import AppConfig, SafetyConfig, load_config
from src.safety_policy import (
    KILL_CHANNEL,
    KILL_PWM_THRESHOLD,
    MODE_CHANNEL,
    MODE_GUIDED_PWM_THRESHOLD,
    SafetyDecision,
    SafetyState,
    evaluate_safety,
)
from src.state import VehicleState

SCENARIOS = [
    "normal",
    "guided_allowed",
    "takeover_revoked",
    "rc_stale",
    "link_lost",
    "kill",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate safety events in dry-run mode.",
    )
    parser.add_argument(
        "--config",
        default="config/dry_run.yaml",
        help="Path to YAML configuration file (default: config/dry_run.yaml).",
    )
    parser.add_argument(
        "--scenario",
        choices=["all", *SCENARIOS],
        default="all",
        help="Which safety scenario to simulate (default: all).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)
    scenarios = SCENARIOS if args.scenario == "all" else [args.scenario]

    print(f"profile: {cfg.profile_name}")
    print(f"dry_run: {cfg.dry_run}")
    print(f"safety.enabled: {cfg.safety.enabled}")
    print(f"safety.allow_arm: {cfg.safety.allow_arm}")
    print()

    now = time.time()
    for name in scenarios:
        snapshot = _build_snapshot(name, now)
        decision = evaluate_safety(
            snapshot=snapshot,
            now_monotonic=now,
            cfg=cfg.safety,
        )
        _print_scenario(name, snapshot, decision)
        print()

    return 0


def _build_snapshot(scenario: str, now: float) -> VehicleState:
    """Build a VehicleState snapshot that triggers the given safety scenario."""
    base = VehicleState(
        last_heartbeat_ts=now,
        armed=False,
        mode="GUIDED",
        system_status="dry_run",
        rc_last_update_ts=now,
        rc_channels={
            MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD + 100,  # 1600 -> GUIDED
            KILL_CHANNEL: 1000,  # below kill threshold
        },
    )

    if scenario == "normal":
        return base

    if scenario == "guided_allowed":
        return replace(base, mode="GUIDED", rc_channels={MODE_CHANNEL: 1600})

    if scenario == "takeover_revoked":
        return replace(
            base,
            rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD - 100},  # 1400
        )

    if scenario == "rc_stale":
        return replace(base, rc_last_update_ts=now - 10.0)

    if scenario == "link_lost":
        return replace(base, last_heartbeat_ts=now - 10.0)

    if scenario == "kill":
        return replace(
            base,
            rc_channels={
                MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD + 100,
                KILL_CHANNEL: KILL_PWM_THRESHOLD + 100,  # 1900
            },
        )

    return base


def _print_scenario(
    name: str,
    snapshot: VehicleState,
    decision: SafetyDecision,
) -> None:
    print(f"--- scenario: {name} ---")
    print(f"  state:            {decision.state.value}")
    print(f"  reason:           {decision.reason}")
    print(f"  cancel_mission:   {decision.should_cancel_mission}")
    print(f"  inhibit_motion:   {decision.should_inhibit_motion}")

    actions = decision.actions
    if actions:
        for i, a in enumerate(actions):
            print(f"  action[{i}]:        {a.action}")
            print(f"  action[{i}].reason: {a.reason}")
            print(f"  action[{i}].once:   {a.once_key}")
    else:
        print("  actions:          (none)")

    print(f"  snapshot.mode:    {snapshot.mode}")
    print(f"  snapshot.armed:   {snapshot.armed}")
    ch5 = snapshot.rc_channels.get(MODE_CHANNEL)
    ch7 = snapshot.rc_channels.get(KILL_CHANNEL)
    print(f"  RC ch5 (mode):    {ch5}")
    print(f"  RC ch7 (kill):    {ch7}")
    print(f"  hb_age_s:         {_fmt_age(snapshot.last_heartbeat_ts)}")
    print(f"  rc_age_s:         {_fmt_age(snapshot.rc_last_update_ts)}")


def _fmt_age(ts: float | None) -> str:
    if ts is None:
        return "None"
    return f"{time.time() - ts:.2f}s ago"


if __name__ == "__main__":
    raise SystemExit(main())
