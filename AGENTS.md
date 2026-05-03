# Cooper_drone safety rules

## Project mission

Cooper_drone is a pymavlink-only companion-computer framework for ArduPilot flight controllers.

This safety-hardening phase must make the codebase safe as a foundation. Do not implement new vision algorithms or advanced movement solutions in this phase.

## Non-negotiable safety rules

- Do not use DroneKit.
- Do not let mission, solutions, or Vision code call raw MAVLink APIs.
- Flight-changing commands must go through the low-level control/movement layer.
- Motion-producing commands must pass through MotionGate.
- Safety actions such as stop, land, brake, loiter, RTL, and disarm must be callable even when normal motion output is inhibited.
- Safety priority must be preserved:
  KILL > MANUAL_TAKEOVER / TAKEOVER_REVOKED > LINK_LOST > RC_STALE > GUIDED_ALLOWED > NORMAL
- KILL must not silently perform an in-air force-disarm unless explicitly configured.
- The default KILL action should be conservative, e.g. stop + LAND or stop + BRAKE/LAND, not force-disarm.
- RC_STALE must have its own configured action instead of implicitly reusing revoke_action.
- Every safety state transition must be logged.
- Every safety action attempt must be logged with result and error, if any.
- Safety policy must be testable without threads or real MAVLink.

## Architecture target

Separate safety into these roles:

- safety policy: pure decision logic, no threads and no MAVLink side effects
- safety supervisor: polling loop and state machine
- safety action executor: executes stop/land/brake/loiter/RTL/disarm actions through the control layer
- motion gate: stores whether normal motion outputs are allowed
- safety event log: structured audit trail for safety transitions and action attempts

## Validation commands

Run after every code change:

```bash
pytest -q
python -m src.main --config tests/data/cfg_dry_run.yaml --dry-run --mission none --standby-seconds 0.1
python -m src.main --config tests/data/cfg_dry_run.yaml --dry-run --mission takeoff_and_hover
```

If the current repository uses root-level task programs instead of `python -m src.main`, run the equivalent dry-run task commands documented in README.

## Do not do in this phase

- Do not implement new computer vision features.
- Do not implement new advanced movement routes.
- Do not add model files, YOLO, ONNX, OpenCV tracking, or camera pipelines.
- Do not change flight behavior unless the task explicitly says it is a safety fix.
- Do not delete existing tests without adding stronger safety tests.
