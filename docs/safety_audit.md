# Cooper_drone safety audit

## 1. Current safety data flow

Current code structure uses these paths:

```text
MAVLink HEARTBEAT / GLOBAL_POSITION_INT / SYS_STATUS / RC_CHANNELS
  -> MavlinkConnection receiver thread
  -> VehicleStateCache.update_from_message()
  -> AppContext.state_provider / MavlinkConnection.state_snapshot()
  -> SafetySupervisor polling loop
  -> evaluate_safety() pure decision
  -> CancellationToken.cancel()
  -> MotionGate.inhibit() / MotionGate.clear()
  -> safety.jsonl transition records
```

Mission and task flow:

```text
root task script
  -> build_app_context()
  -> SolutionContext
  -> src/solutions/*
  -> DroneMovement
  -> MotionGate checks for normal motion outputs
  -> MavlinkCommandService / setpoint send path
```

Setpoint flow:

```text
BodyVelocityStreamer.update()
  -> background loop
  -> DroneMovement.send_body_velocity()
  -> MotionGate.assert_allowed()
  -> MavlinkCommandService.send_body_velocity_setpoint()
```

Important observation: the current safety supervisor only cancels missions, inhibits or clears normal motion, and records `failsafe_action`. It does not execute a safety action such as `stop_motion`, `land`, `brake`, `loiter`, RTL, or disarm.

## 2. Current safety states and priority

Current `SafetyState` values in `src/safety.py`:

- `NORMAL`
- `GUIDED_ALLOWED`
- `RC_STALE`
- `LINK_LOST`
- `MANUAL_TAKEOVER`
- `KILL`

There is no current `TAKEOVER_REVOKED` state. The nearest implemented equivalent is `MANUAL_TAKEOVER`.

Actual implemented priority in `evaluate_safety()` is:

```text
KILL > MANUAL_TAKEOVER > LINK_LOST > RC_STALE > GUIDED_ALLOWED > NORMAL
```

Requested target priority is:

```text
KILL > TAKEOVER_REVOKED > LINK_LOST > RC_STALE > GUIDED_ALLOWED > NORMAL
```

Result: priority is mostly aligned if `MANUAL_TAKEOVER` is treated as the current equivalent of `TAKEOVER_REVOKED`. The explicit `TAKEOVER_REVOKED` state does not exist yet.

## 3. Current safety issue list

1. Safety decision and side effects are partly separated.
   - `evaluate_safety()` is pure and testable.
   - `SafetySupervisor._apply_decision()` mixes state-machine transition handling, mission cancellation, motion gate changes, and safety logging.
   - There is no separate safety action executor.

2. Safety decision can be unit-tested without threads.
   - `evaluate_safety()` is already pure.
   - Existing tests cover KILL, MANUAL_TAKEOVER, LINK_LOST, RC_STALE, and GUIDED_ALLOWED without starting the supervisor thread.
   - Supervisor behavior still uses thread sleep in tests.

3. Motion inhibit is not a module-level global variable.
   - `MotionGate` is an instance object held by `AppContext`.
   - It is thread-safe and not global.

4. Safety actions are not implemented as a dedicated action layer.
   - Normal and safety movement methods exist in `DroneMovement`.
   - Mission recovery code directly calls `ctx.movement.land()` and `ctx.movement.stop_motion()`.
   - The safety supervisor does not call control actions at all; it only logs `failsafe_action`.

5. KILL does not currently force-disarm by default.
   - Config default is `failsafe_action: "land"`.
   - KILL currently only returns a decision with that action; no action is executed.
   - This avoids immediate force-disarm risk, but also means KILL does not actively stop motion or command landing.

6. RC_STALE reuses the single global `failsafe_action`.
   - `SafetyConfig` has only `failsafe_action`.
   - There is no independent `rc_stale_action`.
   - There is no independent takeover revoke action.

7. LINK_LOST does not stop existing velocity setpoints.
   - LINK_LOST cancels mission and inhibits normal motion.
   - Existing `BodyVelocityStreamer` catches motion gate errors and continues running unless stopped by mission teardown.
   - There is no supervisor-owned `stop_motion()` attempt on LINK_LOST.

8. Safety action failure retry/result logging is missing.
   - Since the supervisor does not execute actions, there is no action result, error, retry count, or final status log.
   - Movement methods log command attempts, but those logs are not tied to safety state transitions.

9. Safety state recovery can trigger future transitions, but repeated actions are not modeled.
   - The supervisor logs and cancels only when `decision.state != previous_state`.
   - If an unsafe state persists, no repeated safety action attempt occurs.
   - If state returns to normal and later becomes unsafe again, it can trigger again.
   - There is no explicit action attempt state machine with retry/backoff.

10. Safety state transitions have structured logs.
    - `SafetySupervisor` writes `safety_state_changed` to `safety.jsonl`.
    - It also writes `failsafe_action_recorded`.
    - It does not log action execution result because actions are not executed.

11. `main._safe_exit` conflict was not found.
    - No `src/main.py` or `_safe_exit` was found in the current structure.
    - Root task scripts use `try/finally` and `AppContext.close()`.
    - There is no current direct conflict, but shutdown safety behavior is also not centralized as a safety action boundary.

12. Tests are mixed: policy tests are good, supervisor tests still rely on thread sleep.
    - `test_safety.py` includes pure `evaluate_safety()` tests.
    - `SafetySupervisor` start/stop test uses `time.sleep(0.05)`.
    - There are no pure tests for a safety action executor because that component does not exist yet.

## 4. Risk grading

### P0: real-vehicle safety risks, fix first

1. LINK_LOST and KILL do not actively execute stop/land/brake/loiter actions.
   - Current behavior cancels mission and inhibits normal motion, but a previously running streamer or vehicle state may not receive an explicit safe action.
   - For LINK_LOST, at minimum the system should attempt `stop_motion()` and then a configured conservative action.

2. No dedicated safety action executor with result/error audit.
   - Safety actions are currently only recorded as intended action names.
   - Real-vehicle safety requires action attempts to be executed through the control layer and logged with result or error.

3. Persistent unsafe states do not retry safety actions.
   - If an action fails or is never attempted, the current supervisor will not retry while the state remains unchanged.
   - This is risky for LINK_LOST, KILL, and manual takeover/revoke cases.

### P1: framework reliability risks, must fix

1. Single global `failsafe_action` is too coarse.
   - RC_STALE, LINK_LOST, KILL, and takeover-related states should have independently configured actions.
   - RC_STALE currently reuses `failsafe_action`, which may be inappropriate.

2. No explicit `TAKEOVER_REVOKED` state.
   - The project rule now names `MANUAL_TAKEOVER / TAKEOVER_REVOKED`.
   - Current code only has `MANUAL_TAKEOVER`.
   - If revoked-control semantics differ from manual takeover detection, this needs a distinct state and tests.

3. Safety supervisor combines transition handling, gate mutation, mission cancellation, and logging.
   - This makes action retry, audit, and policy evolution harder.
   - A supervisor state machine should delegate decisions to pure policy and actions to an executor.

4. `BodyVelocityStreamer` keeps trying after motion inhibition.
   - It logs failures and remains alive, which is thread-safe, but there is no supervisor-owned stop of the streamer.
   - A mission cancellation path must reliably stop streamers via mission teardown.

### P2: maintainability and testability issues, should fix

1. Supervisor tests use sleep.
   - Keep short-timeout thread tests, but add more pure state-machine tests without real sleeps.

2. Safety action attempts are not represented as structured domain objects.
   - Add `SafetyAction`, `SafetyActionResult`, or equivalent dataclasses.

3. Safety config does not express policy clearly enough.
   - Add explicit per-state action settings and conservative defaults.

4. Shutdown safety policy is not centralized.
   - Root tasks close resources, but there is no single documented safety-exit path that decides whether to stop, land, brake, or only close logs.

## 5. Phased safety-only repair plan

### Phase A: split safety policy from supervisor state machine

- Modification scope:
  - `src/safety.py` or new `src/safety_policy.py`
  - tests for policy only
- Behavior change:
  - No flight behavior change yet.
  - Preserve current priority and current decisions.
- Tests:
  - Pure policy tests for KILL, TAKEOVER_REVOKED/MANUAL_TAKEOVER, LINK_LOST, RC_STALE, GUIDED_ALLOWED, NORMAL.
  - Explicit test that policy has no movement, MAVLink, or thread side effects.
- Verification commands:
  ```bash
  pytest -q
  python task_healthcheck.py --config config/dry_run.yaml --dry-run
  python task_takeoff_hover.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
  ```

### Phase B: add safety action executor

- Modification scope:
  - new safety action executor module
  - `SafetySupervisor` wiring
  - safety tests with fake movement/control layer
- Behavior change:
  - Unsafe transitions can execute configured conservative safety actions.
  - Actions must go through `DroneMovement` or the low-level control layer.
  - No raw MAVLink in safety code.
- Tests:
  - LINK_LOST attempts `stop_motion()` and then configured action.
  - KILL default does not force-disarm.
  - KILL uses conservative default such as stop + land or stop + brake/land.
  - Action result and error are logged.
  - Action failure does not crash supervisor.
- Verification commands:
  ```bash
  pytest -q
  python task_healthcheck.py --config config/dry_run.yaml --dry-run
  ```

### Phase C: add per-state safety actions to config

- Modification scope:
  - config dataclasses
  - YAML profiles
  - config validation tests
- Behavior change:
  - Replace or supplement single `failsafe_action` with per-state actions:
    - `kill_action`
    - `link_lost_action`
    - `rc_stale_action`
    - `manual_takeover_action`
    - `takeover_revoked_action`, if implemented
  - Keep defaults conservative.
- Tests:
  - RC_STALE no longer reuses revoke or global action implicitly.
  - Invalid action names are rejected.
  - Dry-run resolved config is JSON/YAML serializable.
- Verification commands:
  ```bash
  pytest -q
  python scripts/print_config.py --config config/dry_run.yaml
  ```

### Phase D: add retry and repeated-action behavior

- Modification scope:
  - safety action executor
  - supervisor state machine
  - structured safety audit logging
- Behavior change:
  - Persistent unsafe states can retry configured actions with controlled backoff.
  - Recovery to NORMAL/GUIDED_ALLOWED resets action attempt state where safe.
- Tests:
  - Action failure logs error and retries.
  - Repeated same unsafe state does not spam logs at uncontrolled rates.
  - Returning to normal and becoming unsafe again triggers a new action sequence.
- Verification commands:
  ```bash
  pytest -q
  ```

### Phase E: strengthen mission teardown and streamer cancellation

- Modification scope:
  - mission runtime
  - solution teardown hooks
  - setpoint streamer ownership patterns
- Behavior change:
  - On mission cancellation, active streamers are stopped deterministically.
  - `stop_motion()` is attempted during safety cancellation paths.
- Tests:
  - Cancelled vision follow stops `BodyVelocityStreamer`.
  - LINK_LOST cancellation causes mission teardown to stop streamer.
  - No long sleeps in tests; use fake movement and short timeouts.
- Verification commands:
  ```bash
  pytest -q
  python task_vision_follow.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
  ```

## 6. Audit summary

The codebase already has several good safety foundations:

- Pure `evaluate_safety()` decision function exists.
- `MotionGate` is instance-based and thread-safe.
- Normal velocity and takeoff paths pass through `MotionGate`.
- Safety movement methods such as land, brake, loiter, disarm, and stop are available in `DroneMovement`.
- Safety transitions are logged in structured JSONL.
- Dry-run avoids real MAVLink connection.

The main missing safety-hardening piece is an explicit, tested safety action executor with per-state actions, conservative defaults, action result/error audit logs, and retry behavior. This should be fixed before real-vehicle autonomous testing.
