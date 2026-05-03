# Cooper_drone project rules

## Mission

This repository is a pymavlink-based companion-computer control framework for an ArduPilot Copter vehicle.

Target hardware:
- Companion computer: STM32MP257 Linux board
- Flight controller: Matek H743 V3 running ArduPilot
- MAVLink transport: UART or UDP depending on config

## Required repository layout

Root-level mission scripts must be runnable directly from VSCode.

Expected layout:

```text
Cooper_drone/
├── config/
├── src/
│   ├── Vision/
│   └── solutions/
├── test/
├── task_takeoff_hover.py
├── task_waypoint_square.py
├── task_vision_follow.py
└── task_healthcheck.py
```

## Hard rules

- Use pymavlink only.
- Do not use DroneKit.
- Do not let root task scripts call raw MAVLink.
- Do not let `src/solutions/` call raw MAVLink.
- Only low-level MAVLink modules may touch `pymavlink.mavutil`.
- Movement commands must pass through:
  1. configuration limits
  2. safety gate
  3. logging
  4. MAVLink command service or setpoint streamer
- Mission code must be cancellable.
- Safety logic must work even if a mission script is stuck or raises an exception.
- Dry-run must never send real MAVLink movement commands.
- All flight-state-changing commands must be logged.

## Safety priority

Safety priority must be:

```text
KILL > MANUAL_TAKEOVER > LINK_LOST > RC_STALE > GUIDED_ALLOWED > NORMAL
```

## Required validation commands

After every stage, run:

```bash
pytest -q
python task_healthcheck.py --config config/dry_run.yaml --dry-run
python task_takeoff_hover.py --config config/dry_run.yaml --dry-run
```

## Style

- All code content, comments, and logs must be written in English to avoid encoding issues.
- Prefer simple Python.
- Prefer dataclasses for config and state.
- Avoid hidden global mutable state.
- Threaded loops must have clean stop methods.
- Every background thread must be tested with short timeouts.
- All public APIs must have docstrings.
