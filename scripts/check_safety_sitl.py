"""Read-only safety state monitor — connects to a real MAVLink endpoint.

This script connects, reads heartbeat and RC_CHANNELS messages, evaluates
the safety policy, and prints the current SafetyState every polling cycle.
It NEVER sends commands and NEVER arms.

Usage:
  python scripts/check_safety_sitl.py --config config/sitl.yaml --poll 10
  python scripts/check_safety_sitl.py --config config/sitl.yaml --dry-run
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import replace
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config_loader import SafetyConfig, load_config
from src.logging_runtime import RuntimeLogger
from src.mavlink_connection import MavlinkConnection
from src.safety_policy import (
    KILL_CHANNEL,
    KILL_PWM_THRESHOLD,
    MODE_CHANNEL,
    MODE_GUIDED_PWM_THRESHOLD,
    SafetyState,
    evaluate_safety,
)
from src.state import VehicleState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only safety state monitor for SITL / bench testing.",
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Path to YAML configuration file.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Simulate safety events without a real connection.",
    )
    parser.add_argument(
        "--poll",
        type=int,
        default=0,
        help="Number of polling cycles (default: 0 = run until Ctrl+C).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg = load_config(args.config)

    print(f"profile: {cfg.profile_name}")
    print(f"dry_run: {cfg.dry_run}")
    print(f"connection: {cfg.mavlink.connection_string}")
    print(f"safety.enabled: {cfg.safety.enabled}")
    print(f"safety.allow_arm: {cfg.safety.allow_arm}")
    print(f"safety poll_hz: {cfg.safety.poll_hz}")
    print()

    if args.dry_run or cfg.dry_run:
        return _run_dry(cfg.safety, args.poll)

    return _run_live(cfg, args.poll)


# ---------------------------------------------------------------------------
# Live MAVLink path
# ---------------------------------------------------------------------------


def _run_live(cfg, max_cycles: int) -> int:
    logger = RuntimeLogger(
        run_id="safety-check",
        run_dir=Path("."),
        app_logger=logging.getLogger("check_safety_sitl"),
    )
    connection = MavlinkConnection(cfg.mavlink, logger)

    try:
        connection.connect()
        state = connection.wait_heartbeat()
        print(f"heartbeat received: mode={state.mode} armed={state.armed}")
        connection.start_receiver()
    except Exception as exc:
        print(f"connection failed: {exc}")
        return 1

    previous_state: SafetyState | None = None
    cycle = 0
    poll_interval = 1.0 / max(cfg.safety.poll_hz, 1.0)

    try:
        while max_cycles == 0 or cycle < max_cycles:
            cycle += 1
            snapshot = connection.state_snapshot()
            now = time.time()
            decision = evaluate_safety(
                snapshot=snapshot,
                now_monotonic=now,
                cfg=cfg.safety,
                previous_state=previous_state,
            )

            if decision.state != previous_state:
                _print_transition(decision, previous_state, snapshot)
            else:
                _print_status(cycle, decision.state, snapshot)

            previous_state = decision.state
            time.sleep(poll_interval)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        connection.stop_receiver()
        connection.close()

    return 0


def _print_transition(decision, previous, snapshot) -> None:
    prev_label = previous.value if previous else "initial"
    print(f"\n>>> STATE CHANGE: {prev_label} -> {decision.state.value}")
    print(f"    reason:         {decision.reason}")
    print(f"    cancel_mission: {decision.should_cancel_mission}")
    print(f"    inhibit_motion: {decision.should_inhibit_motion}")
    for a in decision.actions:
        print(f"    action:         {a.action}  (reason={a.reason})")

    ch5 = snapshot.rc_channels.get(MODE_CHANNEL, "—")
    ch7 = snapshot.rc_channels.get(KILL_CHANNEL, "—")
    print(f"    mode: {snapshot.mode}  armed={snapshot.armed}  ch5={ch5}  ch7={ch7}")


def _print_status(cycle: int, state: SafetyState, snapshot: VehicleState) -> None:
    ch5 = snapshot.rc_channels.get(MODE_CHANNEL, "—")
    ch7 = snapshot.rc_channels.get(KILL_CHANNEL, "—")
    print(
        f"[{cycle:04d}] state={state.value:<20s} "
        f"mode={snapshot.mode or '?':<12s} armed={snapshot.armed} "
        f"ch5={ch5} ch7={ch7}"
    )


# ---------------------------------------------------------------------------
# Dry-run simulation path
# ---------------------------------------------------------------------------


def _run_dry(safety_cfg: SafetyConfig, max_cycles: int) -> int:
    """Simulate a sequence of safety events without real MAVLink."""
    print("dry-run: simulating safety event sequence\n")

    now = time.time()
    snapshots = _dry_snapshots(now)
    previous_state: SafetyState | None = None

    for label, snapshot in snapshots:
        decision = evaluate_safety(
            snapshot=snapshot,
            now_monotonic=now,
            cfg=safety_cfg,
            previous_state=previous_state,
        )

        prev_label = previous_state.value if previous_state else "initial"
        print(f">>> {prev_label} -> {decision.state.value:<18s}  ({label})")
        print(f"    reason:         {decision.reason}")
        print(f"    cancel_mission: {decision.should_cancel_mission}")
        print(f"    inhibit_motion: {decision.should_inhibit_motion}")
        for a in decision.actions:
            print(f"    action:         {a.action}  (once_key={a.once_key})")
        print()

        previous_state = decision.state
        time.sleep(0.3)

    return 0


def _dry_snapshots(now: float):
    """Yield (label, VehicleState) pairs that exercise each safety transition."""
    guided = {
        MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD + 100,
        KILL_CHANNEL: 1000,
    }
    base = VehicleState(
        last_heartbeat_ts=now,
        armed=False,
        mode="GUIDED",
        system_status="dry_run",
        rc_last_update_ts=now,
        rc_channels=guided,
    )
    yield "baseline NORMAL", base
    yield (
        "TAKEOVER_REVOKED: mode channel low",
        replace(base, rc_channels={MODE_CHANNEL: MODE_GUIDED_PWM_THRESHOLD - 100}),
    )
    yield "recovery to NORMAL", base
    yield (
        "RC_STALE: no recent RC",
        replace(base, rc_last_update_ts=now - 10.0),
    )
    yield "recovery to NORMAL", base
    yield (
        "LINK_LOST: no recent heartbeat",
        replace(base, last_heartbeat_ts=now - 10.0),
    )
    yield "recovery to NORMAL", base
    yield (
        "KILL: channel 7 high",
        replace(base, rc_channels={**guided, KILL_CHANNEL: KILL_PWM_THRESHOLD + 100}),
    )
    yield "recovery to NORMAL", base


if __name__ == "__main__":
    raise SystemExit(main())
