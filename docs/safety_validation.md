# Cooper_drone Safety Validation

This document defines the safety validation procedures for the Cooper_drone
autonomous flight system.  Validation is **phased**: dry-run first, SITL next,
then no-propeller bench testing, and only then real flight.

**Every phase must pass before moving to the next.**

---

## 1. Dry‑Run Safety Validation

Dry-run mode uses the config profile `config/dry_run.yaml`.  No MAVLink
connection is opened — all commands are captured in memory and logged to
JSONL files under `logs/<run_id>/`.

### 1.1 Run the full test suite

```bash
pytest -q
```

Expected: **all 149 tests pass** with no failures or errors.

### 1.2 Run each root task script in dry-run mode

```bash
# Healthcheck (standby — no flight mission)
python task_healthcheck.py --config config/dry_run.yaml --dry-run

# Takeoff and hover (RUNNING mission)
python task_takeoff_hover.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1

# Waypoint square (RUNNING mission)
python task_waypoint_square.py --config config/dry_run.yaml --dry-run --altitude 1 --side 1 --speed 0.5

# Vision follow (RUNNING mission)
python task_vision_follow.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
```

Expected output for every task: `* finished: log_dir=logs/...`

### 1.3 Verify safe-exit logs

After each dry-run task, check that the safety JSONL includes the
appropriate `safe_exit_*` events:

```bash
# For a RUNNING mission (takeoff_hover / waypoint_square / vision_follow):
grep "safe_exit" logs/<run_id>/events.jsonl
# Expected:
#   safe_exit_mission_cancelled  (if cancelled)
#   safe_exit_dry_run_skipped    (dry-run skips flight actions)

# For a standby task (healthcheck):
grep "safe_exit" logs/<run_id>/events.jsonl
# Expected:
#   safe_exit_dry_run_skipped    exit_action=loiter
```

### 1.4 Verify config validation rejects bad actions

```bash
# Should FAIL — invalid action name
python scripts/print_config.py --config test/fixtures/config_loader/invalid_failsafe.yaml 2>&1
```

Expected: `ConfigError` with message mentioning the invalid action.

### 1.5 Verify safety event serialisation

```bash
pytest -q test/test_safety_events.py
```

Expected: all tests pass — confirms `SafetyEvent.to_dict()` / `.to_json()`
and all recorder convenience helpers work correctly.

---

## 2. SITL Safety Validation

SITL (Software-In-The-Loop) validation uses ArduPilot SITL with a UDP
connection.  The SITL config is `config/sitl.yaml`.

**Prerequisites:**

- ArduPilot SITL running and listening on `udp:127.0.0.1:14550`.
- The config disables arming (`allow_arm: false`) — no motors will spin.

### 2.1 Start ArduPilot SITL

```bash
# Example (adjust vehicle and location as needed):
sim_vehicle.py -v Copter --console --map -L HEX
```

### 2.2 Print resolved SITL config

```bash
python scripts/print_config.py --config config/sitl.yaml
```

Confirm:
- `dry_run: false`
- `safety.enabled: true`
- `safety.allow_arm: false`
- `safety.failsafe_action: brake`
- `mavlink.connection_string: udp:127.0.0.1:14550`

### 2.3 Verify heartbeat connection

```bash
python scripts/check_serial_link.py --config config/sitl.yaml
```

Expected output:
```
target_system: ...
target_component: ...
mode: ...
armed: ...
```

If this fails: check that SITL is running and the UDP port matches.

### 2.4 Run healthcheck with SITL

```bash
python task_healthcheck.py --config config/sitl.yaml
```

Expected: prints `mode: ...` and `armed: False`.  No commands are sent.

### 2.5 Monitor safety state with the SITL safety script

```bash
python scripts/check_safety_sitl.py --config config/sitl.yaml --poll 10
```

This script connects, reads heartbeat and RC channels, evaluates the safety
policy, and prints the current `SafetyState` every polling cycle.  It **never
sends commands** and **never arms**.

### 2.6 Trigger TAKEOVER_REVOKED

In the SITL console or via MAVProxy, switch the flight mode away from
GUIDED (e.g. `mode STABILIZE` or `mode LOITER`).

Expected within one poll interval:

```text
state_transition: NORMAL -> takeover_revoked
  reason: guided control revoked
motion_inhibited: reason=guided control revoked
mission_cancelled: reason=guided control revoked
action_completed: action=loiter
  reason=guided control revoked
```

### 2.7 Trigger RC_STALE

Configure SITL to stop sending RC_CHANNELS messages (or disconnect the
joystick / RC simulator in MAVProxy).

Expected within `rc_stale_timeout_s` (default 1.0 s):

```text
state_transition: NORMAL -> rc_stale
  reason: RC input is stale
motion_inhibited: reason=RC input is stale
mission_cancelled: reason=RC input is stale
action_completed: action=brake
  reason=RC input is stale
```

### 2.8 Trigger LINK_LOST

Stop the SITL process (or disconnect the UDP link).

Expected within `heartbeat_loss_timeout_s` (default 2.0 s):

```text
state_transition: NORMAL -> link_lost
  reason: heartbeat link lost
motion_inhibited: reason=heartbeat link lost
mission_cancelled: reason=heartbeat link lost
action_completed: action=land
  reason=heartbeat link lost
```

### 2.9 Trigger KILL

In SITL, set RC channel 7 (kill switch) PWM to ≥ 1800:

```bash
# In MAVProxy console:
rc 7 1900
```

Expected within one poll interval:

```text
state_transition: NORMAL -> kill
  reason: kill channel active
motion_inhibited: reason=kill channel active
mission_cancelled: reason=kill channel active
action_completed: action=land
  reason=kill channel active
```

### 2.10 Confirm action messages are sent on the MAVLink wire

Use `mavproxy` or Wireshark to verify that the expected MAVLink messages
appear when safety actions fire:

| Action   | Expected MAVLink message |
|----------|--------------------------|
| `stop`   | `SET_POSITION_TARGET_LOCAL_NED` (zero velocity) |
| `land`   | `MAV_CMD_NAV_LAND` (COMMAND_LONG) |
| `loiter` | `MAV_CMD_DO_SET_MODE` (LOITER = 5) |
| `brake`  | `MAV_CMD_DO_SET_MODE` (BRAKE = 17) |

### 2.11 Run dry-run safety check script

Even without SITL running, the dry-run variant exercises the full safety
evaluation pipeline:

```bash
python scripts/check_safety_dry_run.py --scenario all
```

Expected: prints expected safety decisions for KILL, TAKEOVER_REVOKED,
LINK_LOST, RC_STALE, GUIDED_ALLOWED, and NORMAL states.

### 2.12 SITL validation checklist

- [ ] Heartbeat received from SITL
- [ ] RC channels readable (channels 5 and 7 confirmed)
- [ ] TAKEOVER_REVOKED triggers on mode change away from GUIDED
- [ ] RC_STALE triggers when RC stops updating
- [ ] LINK_LOST triggers when heartbeat stops
- [ ] KILL triggers on channel 7 PWM ≥ 1800
- [ ] LAND / LOITER / BRAKE action commands appear on MAVLink wire
- [ ] `pytest -q` passes (no SITL-dependent tests in the default suite)

---

## 3. No‑Propeller Bench Validation

Bench testing uses the hardware flight controller (e.g. STM32MP257) with
**propellers removed** and the airframe physically restrained.

### 3.1 Hardware preparation

1. **Remove all propellers.**  This is mandatory — do not skip.
2. **Restrain the airframe.**  Use straps, clamps, or a fixed mount so the
   vehicle cannot move.
3. Connect the flight controller UART to the host computer.
4. Power the vehicle through a **current-limited bench supply** if available.
   Otherwise use the normal flight battery.
5. Place a fire extinguisher within reach.

### 3.2 Radio controller verification

Before connecting to the flight controller, verify the RC transmitter:

1. Power on the RC transmitter.
2. Confirm the **mode channel** (channel 5) PWM is displayed:
   - ≥ 1500 µs → GUIDED allowed
   - < 1500 µs → GUIDED revoked (triggers TAKEOVER_REVOKED)
3. Confirm the **kill switch channel** (channel 7) PWM is displayed:
   - < 1800 µs → normal operation
   - ≥ 1800 µs → KILL active
4. Test the kill switch: toggle it and confirm the PWM changes
   immediately on the transmitter display.

### 3.3 Serial connection

Use the hardware UART config:

```bash
# Verify the config
python scripts/print_config.py --config config/stm32mp257_uart.yaml

# Check the serial link
python scripts/check_serial_link.py --config config/stm32mp257_uart.yaml
```

Expected output:
```
target_system: 1
target_component: 1
mode: STABILIZE  (or whatever the current mode is)
armed: False
```

### 3.4 Read RC channels from the flight controller

```bash
python scripts/check_safety_sitl.py --config config/stm32mp257_uart.yaml --poll 5
```

Confirm the printed RC channels match the transmitter display:
- Channel 5 (mode): visible and updating
- Channel 7 (kill): visible and updating

### 3.5 Safety trigger tests (bench)

Perform each trigger **with propellers off** and the airframe restrained.

#### 3.5.1 TAKEOVER_REVOKED

1. Start the safety monitor: `python scripts/check_safety_sitl.py --config config/stm32mp257_uart.yaml --poll 5`
2. On the RC transmitter, switch the flight mode channel below 1500 µs
   (or change the flight mode to STABILIZE).
3. Confirm the monitor prints `takeover_revoked` and the configured action
   (`loiter`).

#### 3.5.2 RC_STALE

1. Start the safety monitor.
2. Power off the RC transmitter (or disable RF).
3. Within 1 second, the monitor should print `rc_stale` and `brake`.

#### 3.5.3 KILL

1. Start the safety monitor.
2. Activate the kill switch (channel 7 PWM ≥ 1800).
3. Confirm the monitor prints `kill` and the configured action (`land`).

#### 3.5.4 LINK_LOST — limited test

Bench testing LINK_LOST requires physically disconnecting the UART or
power-cycling the flight controller.  This is acceptable for validation
but should be done carefully:

1. Start the safety monitor.
2. Disconnect the UART cable from the host.
3. Within 2 seconds, the monitor should print `link_lost` and `land`.

### 3.6 Tasks that MUST NOT be run on the bench

Do **not** run any task that sends movement commands with propellers on
or with arming enabled:

- `task_takeoff_hover.py` — would command takeoff
- `task_waypoint_square.py` — would command takeoff + movement
- `task_vision_follow.py` — would command takeoff + velocity setpoints

The bench config `stm32mp257_uart.yaml` has `allow_arm: false`, so arming
will be rejected even if a script attempts it.  **Do not change this
setting on the bench.**

### 3.7 Bench validation checklist

- [ ] Propellers removed
- [ ] Airframe physically restrained
- [ ] RC transmitter mode channel (5) verified
- [ ] RC transmitter kill channel (7) verified
- [ ] Serial connection to flight controller confirmed
- [ ] Heartbeat received
- [ ] RC channels readable from flight controller
- [ ] TAKEOVER_REVOKED triggers on mode change
- [ ] RC_STALE triggers on RC transmitter power-off
- [ ] KILL triggers on kill switch activation
- [ ] LINK_LOST triggers on UART disconnect
- [ ] All triggered actions match the configured per-state actions
- [ ] No arming or takeoff commands were sent

---

## 4. Pre‑Real‑Flight Prohibitions

These prohibitions are **mandatory**.  Real flight must not be attempted
until every item is confirmed.

| # | Prohibition | Rationale |
|---|-------------|-----------|
| 1 | **Do not install propellers before safety tests pass.** | Safety actions must be proven functional on the bench first. |
| 2 | **Do not configure KILL as `force_disarm`.** | The default `kill_action: land` and `allow_force_disarm_on_kill: false` are conservative. Mid-air disarm is unrecoverable. |
| 3 | **Do not run GUIDED tasks before confirming RC takeover logic.** | If TAKEOVER_REVOKED does not fire correctly, the pilot cannot regain control by switching modes. |
| 4 | **Do not fly outdoors before confirming LINK_LOST behaviour.** | If the heartbeat timeout does not trigger LAND, a lost link results in uncontrolled flight. |
| 5 | **Do not fly with `safety.enabled: false`.** | The safety supervisor is the only in-flight protection layer. |
| 6 | **Do not use `allow_arm: true` until bench validation is complete.** | Arming should only be enabled after all safety triggers have been verified. |
| 7 | **Do not fly without a working kill switch on channel 7.** | The kill switch is the ultimate pilot override. Verify PWM ≥ 1800 µs triggers KILL on the bench before flight. |

---

## 5. Expected Logs Per Safety Trigger

All safety events are written to `safety.jsonl` in the run directory
(`logs/<run_id>/safety.jsonl`).  Each event is a single JSON line with
the fields described below.

### 5.1 TAKEOVER_REVOKED

Trigger: flight mode changed away from GUIDED, or mode channel PWM < 1500.

```
event_type: state_transition
  state: takeover_revoked
  previous_state: normal | guided_allowed
  reason: guided control revoked

event_type: mission_cancelled
  state: takeover_revoked
  reason: guided control revoked

event_type: motion_inhibited
  state: takeover_revoked
  reason: guided control revoked

event_type: action_completed
  state: takeover_revoked
  action: loiter              # configured via revoke_action
  reason: guided control revoked
```

Safety JSONL records (written by `SafetySupervisor`):

```json
{"name":"safety_state_changed","state":"takeover_revoked","reason":"guided control revoked","cancel_mission":true,"inhibit_motion":true,"failsafe_action":"loiter"}
{"name":"failsafe_action_recorded","state":"takeover_revoked","action":"loiter"}
{"name":"safety_action_result","action":"loiter","reason":"guided control revoked","ok":true,...}
```

### 5.2 RC_STALE

Trigger: no RC_CHANNELS update within `rc_stale_timeout_s` (default 1.0 s).

```
event_type: state_transition
  state: rc_stale
  previous_state: normal | guided_allowed
  reason: RC input is stale

event_type: mission_cancelled
  state: rc_stale
  reason: RC input is stale

event_type: motion_inhibited
  state: rc_stale
  reason: RC input is stale

event_type: action_completed
  state: rc_stale
  action: brake               # configured via rc_stale_action
  reason: RC input is stale
```

Safety JSONL records:

```json
{"name":"safety_state_changed","state":"rc_stale","reason":"RC input is stale","cancel_mission":true,"inhibit_motion":true,"failsafe_action":"brake"}
{"name":"failsafe_action_recorded","state":"rc_stale","action":"brake"}
{"name":"safety_action_result","action":"brake","reason":"RC input is stale","ok":true,...}
```

### 5.3 LINK_LOST

Trigger: no heartbeat within `heartbeat_loss_timeout_s` (default 2.0 s).

```
event_type: state_transition
  state: link_lost
  previous_state: normal | guided_allowed | rc_stale
  reason: heartbeat link lost

event_type: mission_cancelled
  state: link_lost
  reason: heartbeat link lost

event_type: motion_inhibited
  state: link_lost
  reason: heartbeat link lost

event_type: action_completed
  state: link_lost
  action: land                # configured via link_lost_action
  reason: heartbeat link lost
```

Safety JSONL records:

```json
{"name":"safety_state_changed","state":"link_lost","reason":"heartbeat link lost","cancel_mission":true,"inhibit_motion":true,"failsafe_action":"land"}
{"name":"failsafe_action_recorded","state":"link_lost","action":"land"}
{"name":"safety_action_result","action":"land","reason":"heartbeat link lost","ok":true,...}
```

### 5.4 KILL

Trigger: RC channel 7 PWM ≥ 1800 µs.

```
event_type: state_transition
  state: kill
  previous_state: normal | guided_allowed | *   # KILL overrides all other states
  reason: kill channel active

event_type: mission_cancelled
  state: kill
  reason: kill channel active

event_type: motion_inhibited
  state: kill
  reason: kill channel active

event_type: action_completed
  state: kill
  action: land                # configured via kill_action
  reason: kill channel active
```

Safety JSONL records:

```json
{"name":"safety_state_changed","state":"kill","reason":"kill channel active","cancel_mission":true,"inhibit_motion":true,"failsafe_action":"land"}
{"name":"failsafe_action_recorded","state":"kill","action":"land"}
{"name":"safety_action_result","action":"land","reason":"kill channel active","ok":true,...}
```

### 5.5 Recovery to NORMAL

When the triggering condition clears, the supervisor transitions back:

```
event_type: state_transition
  state: normal
  previous_state: takeover_revoked | rc_stale | link_lost | kill
  reason: safety checks passed
```

Motion inhibition is cleared.  `_executed_once_keys` are reset so that a
future safety violation triggers a fresh action sequence.

### 5.6 Safe Exit (app shutdown)

When any task script exits, `safe_exit()` runs in the `finally` block.

**RUNNING mission exit** (task_takeoff_hover, task_waypoint_square,
task_vision_follow):

```json
{"name":"safe_exit_mission_cancelled","reason":"..."}       // if cancelled
{"name":"safe_exit_action","action":"stop","ok":true,...}
{"name":"safe_exit_action","action":"land","ok":true,...}   // exit_action
```

**Standby exit** (task_healthcheck):

```json
{"name":"safe_exit_action","action":"stop","ok":true,...}
{"name":"safe_exit_action","action":"loiter","ok":true,...} // standby_exit_action
```

**Dry-run exit** (any task with `--dry-run`):

```json
{"name":"safe_exit_dry_run_skipped","exit_action":"land"}   // or loiter
```

### 5.7 Event field reference

Every `SafetyEvent` in `safety.jsonl` carries:

| Field | Type | Description |
|-------|------|-------------|
| `timestamp_monotonic` | float | `time.monotonic()` when event was created |
| `event_type` | str | One of the 9 event type constants (see below) |
| `state` | str | Current safety state value |
| `previous_state` | str \| null | Previous safety state value |
| `reason` | str | Human-readable trigger reason |
| `heartbeat_age_s` | float \| null | Age of last heartbeat at decision time |
| `rc_age_s` | float \| null | Age of last RC update at decision time |
| `rc_channels` | dict \| null | RC channel snapshot (keys stringified) |
| `actions` | list \| null | Actions associated with this event |
| `error` | str \| null | Error detail for failed actions / supervisor errors |

**Event type constants:**

| Constant | Value |
|----------|-------|
| `EVENT_STATE_TRANSITION` | `state_transition` |
| `EVENT_DECISION_EVALUATED` | `decision_evaluated` |
| `EVENT_ACTION_STARTED` | `action_started` |
| `EVENT_ACTION_COMPLETED` | `action_completed` |
| `EVENT_ACTION_FAILED` | `action_failed` |
| `EVENT_ACTION_SKIPPED_ONCE_KEY` | `action_skipped_once_key` |
| `EVENT_MOTION_INHIBITED` | `motion_inhibited` |
| `EVENT_MISSION_CANCELLED` | `mission_cancelled` |
| `EVENT_SUPERVISOR_ERROR` | `supervisor_error` |

---

## 6. Safety Configuration Reference

### 6.1 Supported action names

| Action | MAVLink effect | Config key example |
|--------|---------------|-------------------|
| `land` | `MAV_CMD_NAV_LAND` | `failsafe_action: land` |
| `loiter` | `MAV_CMD_DO_SET_MODE` (LOITER=5) | `revoke_action: loiter` |
| `brake` | `MAV_CMD_DO_SET_MODE` (BRAKE=17) | `rc_stale_action: brake` |
| `rtl` | `MAV_CMD_DO_SET_MODE` (RTL=6) | `link_lost_action: rtl` |
| `stop` | Zero-velocity setpoint | (used internally by `safe_exit`) |
| `disarm` | `MAV_CMD_COMPONENT_ARM_DISARM` (0) | Requires `allow_disarm: true` |
| `force_disarm` | `MAV_CMD_COMPONENT_ARM_DISARM` (0) | Requires `allow_force_disarm: true` |
| `none` | No action | `failsafe_action: none` |

### 6.2 Per-state action configuration

```yaml
safety:
  enabled: true
  allow_arm: false                 # Must stay false until bench validated

  # Per-state safety actions (all default to conservative values):
  failsafe_action: land            # Fallback for any unconfigured state
  kill_action: land                # KILL state
  link_lost_action: land           # LINK_LOST state
  revoke_action: loiter            # TAKEOVER_REVOKED state
  rc_stale_action: brake           # RC_STALE state

  # Shutdown actions:
  exit_action: land                # Abnormal exit of a RUNNING mission
  standby_exit_action: loiter      # Exit when no mission was running

  # Disarm is NOT a default safety action:
  allow_force_disarm_on_kill: false
```

### 6.3 RC channel mapping

| Channel | Function | Threshold |
|---------|----------|-----------|
| 5 | Flight mode | ≥ 1500 µs = GUIDED allowed; < 1500 µs = TAKEOVER_REVOKED |
| 7 | Kill switch | ≥ 1800 µs = KILL active |

---

## 7. Test Suite Reference

```bash
# Run all tests (does NOT require SITL or hardware):
pytest -q

# Run only safety-related tests:
pytest -q test/test_safety.py test/test_safety_actions.py test/test_safety_policy.py test/test_safety_events.py test/test_main_safety_exit.py

# Run dry-run integration tests:
pytest -q test/test_dry_run_integration.py

# Count tests:
pytest --collect-only -q 2>&1 | tail -1
```

Key test files:

| File | Covers |
|------|--------|
| `test/test_safety_policy.py` | Pure policy decisions for all 6 states |
| `test/test_safety_actions.py` | SafetyActionExecutor — all 8 action types |
| `test/test_safety.py` | SafetySupervisor thread + action integration |
| `test/test_safety_events.py` | SafetyEvent serialisation + recorder |
| `test/test_main_safety_exit.py` | `safe_exit()` — all 6 failure + edge case scenarios |
| `test/test_dry_run_integration.py` | End-to-end dry-run task runs |

---

## 8. Scripts Reference

| Script | Purpose | Sends commands? |
|--------|---------|-----------------|
| `scripts/print_config.py` | Print resolved YAML config | No |
| `scripts/check_serial_link.py` | Verify MAVLink heartbeat | No |
| `scripts/check_safety_sitl.py` | Monitor safety state via real MAVLink | No |
| `scripts/check_safety_dry_run.py` | Simulate safety events (no connection) | No |

---

## 9. Phase Gate Checklist

### Gate A: Dry-run → proceed to SITL

- [ ] `pytest -q` passes (all 149 tests)
- [ ] All 4 task scripts run in dry-run without error
- [ ] Config validation rejects invalid actions
- [ ] Safety event serialisation tests pass
- [ ] `safe_exit` tests pass (23 tests)

### Gate B: SITL → proceed to bench

- [ ] Heartbeat received from SITL
- [ ] All 4 safety triggers (TAKEOVER_REVOKED, RC_STALE, LINK_LOST, KILL) produce correct events
- [ ] Correct MAVLink action messages appear on the wire
- [ ] `pytest -q` still passes (no SITL pollution in test suite)

### Gate C: Bench → proceed to real flight

- [ ] Propellers removed during all bench tests
- [ ] All 4 safety triggers verified on real hardware
- [ ] RC channels confirmed on both transmitter and flight controller
- [ ] Kill switch confirmed functional
- [ ] No arming or takeoff commands were sent during bench testing

### Gate D: Pre-flight final checks

- [ ] All pre-real-flight prohibitions (Section 4) confirmed
- [ ] Propellers installed **only after** all safety tests pass
- [ ] Kill switch tested one final time with propellers on (restrained)
- [ ] Flight area cleared and safety observer briefed
