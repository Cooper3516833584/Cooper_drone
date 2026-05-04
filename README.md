# Cooper_drone

Cooper_drone is a companion-computer framework for an STM32MP257 Linux board controlling an ArduPilot Copter vehicle through MAVLink with `pymavlink`.

Target stack:
- Companion computer: STM32MP257 Linux board
- Flight controller: Matek H743 V3 running ArduPilot Copter
- MAVLink Python library: `pymavlink`

## Install

```bash
python -m pip install -e .[dev]
```

## Run Root Mission Scripts

Root-level task scripts are intended to be runnable directly from VSCode or from a terminal:

```bash
python task_healthcheck.py --config config/dry_run.yaml --dry-run
python task_takeoff_hover.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
python task_waypoint_square.py --config config/dry_run.yaml --dry-run --altitude 1 --side 1 --speed 0.5
python task_vision_follow.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
```

These scripts are entrypoints only. Mission logic must call higher-level services and must not call raw MAVLink directly.

## VSCode 一键运行

1. 打开 Cooper_drone 文件夹。
2. 选择 Python 解释器。
3. 安装依赖。
4. 打开 Run and Debug。
5. 选择 Cooper: healthcheck dry-run。
6. 点击运行。

直接运行 root task 文件时，VSCode workspace root 必须是项目根目录。如果 import 失败，检查 `PYTHONPATH=${workspaceFolder}` 是否生效。真机配置不要误选 dry-run；真机运行前必须拆桨。

## Deployment Path

Use this progression:

```text
dry-run -> SITL -> no-props bench test -> tethered low-altitude test -> controlled real flight
```

Read these documents before moving beyond dry-run:

- `docs/sitl.md`
- `docs/hardware_setup.md`
- `docs/no_props_checklist.md`
- `docs/first_flight_checklist.md`

## Dry Run

Use dry-run before any hardware or SITL run:

```bash
python task_healthcheck.py --config config/dry_run.yaml --dry-run
python task_takeoff_hover.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
```

Dry-run mode must never send real MAVLink movement commands.

## 测试

默认测试：

```bash
pytest -q
```

dry-run 集成测试：

```bash
python task_takeoff_hover.py --config config/dry_run.yaml --dry-run --altitude 1 --duration 0.1
```

SITL 测试：

```bash
pytest -m sitl
```

Default tests do not require a flight controller, a camera, or ArduPilot SITL. Tests marked with `sitl` are excluded from the default test run.

## Deployment Helper Scripts

Print resolved configuration:

```bash
python scripts/print_config.py --config config/stm32mp257_uart.yaml
```

Check a MAVLink link without sending movement commands:

```bash
python scripts/check_serial_link.py --config config/stm32mp257_uart.yaml
```

## Safety Before Real Vehicle Use

Before running on a real vehicle, validate changes in this order:

1. Dry-run
2. SITL
3. No-props bench test
4. Tethered low-altitude test
5. Controlled real flight only after the previous checks pass

Never install propellers during bench testing. 真机首次测试必须拆桨。
