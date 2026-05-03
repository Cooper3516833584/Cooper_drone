# SITL workflow

SITL should be used after dry-run and before any hardware test. It exercises MAVLink transport, ArduPilot mode changes, arming checks, and mission timing without risking a real vehicle.

## Start ArduPilot Copter SITL

Install ArduPilot SITL using the official ArduPilot development environment for your host OS. From an ArduPilot checkout, a typical Copter SITL session is:

```bash
sim_vehicle.py -v ArduCopter --console --map
```

Confirm the MAVLink output endpoint used by SITL. A common default is UDP on `127.0.0.1:14550`.

## Configure Cooper_drone

Edit `config/sitl.yaml` if your SITL endpoint differs:

```yaml
profile_name: sitl
dry_run: false
mavlink:
  connection_string: "udp:127.0.0.1:14550"
```

Do not enable automatic takeoff during configuration checks.

## Run SITL tasks

```bash
python task_healthcheck.py --config config/sitl.yaml
python task_takeoff_hover.py --config config/sitl.yaml --altitude 2 --duration 3
```

## Run SITL tests

SITL tests are excluded from the default test run. Run them explicitly:

```bash
pytest -m sitl
```

## Common issues

- No heartbeat: confirm SITL is running and the endpoint matches `connection_string`.
- Wrong port: check SITL console output for the active UDP port.
- GUIDED mode change fails: check ArduPilot mode availability and vehicle readiness.
- Arm rejected: inspect ArduPilot messages for pre-arm checks.
- EKF, GPS, or Home not ready: wait for SITL initialization or adjust the simulation setup.
