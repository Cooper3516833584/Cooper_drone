# STM32MP257 and Matek H743 V3 hardware setup

This checklist covers companion-computer wiring and configuration checks before bench testing.

## Wiring

- Cross TX and RX: STM32MP257 TX to H743 RX, STM32MP257 RX to H743 TX.
- Connect common ground between the STM32MP257 board and the flight controller.
- Confirm voltage levels before connecting UART pins.
- Identify the Linux serial device. It may be `/dev/ttySTM*`, `/dev/ttyUSB*`, or another board-specific device. Do not assume the example path is correct.
- Confirm Linux serial permissions. The runtime user may need membership in groups such as `dialout`, depending on the distribution.

## ArduPilot serial configuration

Choose the ArduPilot SERIALx port connected to the companion computer and verify:

- `SERIALx_PROTOCOL` is configured for MAVLink.
- `SERIALx_BAUD` matches the Linux-side baud rate.
- MAVLink2 is enabled when required by your ArduPilot configuration.

## Cooper_drone UART config

Edit `config/stm32mp257_uart.yaml` for the actual device and baud:

```yaml
mavlink:
  connection_string: "/dev/ttySTM0"
  baud: 921600
```

The path above is only an example. Confirm the real device name on the STM32MP257 Linux image.

## Deployment notes

For later `systemd` deployment:

- Set the working directory to the project root.
- Set `PYTHONPATH` to the project root.
- Run as a user that can access the serial device.
- Store logs on persistent storage with enough free space.
- Start with healthcheck only before enabling mission services.

真机首次测试必须拆桨。
