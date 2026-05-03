"""MAVLink message parsing helpers."""

from __future__ import annotations

from typing import Any


MAV_MODE_FLAG_SAFETY_ARMED = 128


def heartbeat_to_mode_and_armed(msg: Any) -> tuple[str | None, bool | None]:
    """Parse a HEARTBEAT message into a mode name and armed state."""
    armed = None
    base_mode = _field(msg, "base_mode")
    if base_mode is not None:
        armed = bool(int(base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)

    mode = _field(msg, "flightmode")
    if mode is None:
        mode = _field(msg, "mode")
    if mode is None:
        custom_mode = _field(msg, "custom_mode")
        if custom_mode is not None:
            mode = f"custom_mode:{custom_mode}"

    return mode, armed


def heartbeat_to_system_status(msg: Any) -> str | None:
    """Parse a HEARTBEAT message into a system status string."""
    status = _field(msg, "system_status_name")
    if status is not None:
        return str(status)

    status = _field(msg, "system_status")
    if status is None:
        return None
    return str(status)


def global_position_to_fields(msg: Any) -> dict[str, Any]:
    """Parse GLOBAL_POSITION_INT fields into SI units."""
    fields: dict[str, Any] = {}
    if _field(msg, "relative_alt") is not None:
        fields["relative_alt_m"] = float(_field(msg, "relative_alt")) / 1000.0
    if _field(msg, "lat") is not None:
        fields["lat_deg"] = float(_field(msg, "lat")) / 10_000_000.0
    if _field(msg, "lon") is not None:
        fields["lon_deg"] = float(_field(msg, "lon")) / 10_000_000.0
    if _field(msg, "vx") is not None:
        fields["vx_mps"] = float(_field(msg, "vx")) / 100.0
    if _field(msg, "vy") is not None:
        fields["vy_mps"] = float(_field(msg, "vy")) / 100.0
    if _field(msg, "vz") is not None:
        fields["vz_mps"] = float(_field(msg, "vz")) / 100.0
    return fields


def sys_status_to_fields(msg: Any) -> dict[str, Any]:
    """Parse SYS_STATUS fields into battery state."""
    fields: dict[str, Any] = {}
    voltage_mv = _field(msg, "voltage_battery")
    if voltage_mv is not None and int(voltage_mv) >= 0:
        fields["battery_voltage_v"] = float(voltage_mv) / 1000.0

    battery_remaining = _field(msg, "battery_remaining")
    if battery_remaining is not None and int(battery_remaining) >= 0:
        fields["battery_remaining_pct"] = int(battery_remaining)
    return fields


def rc_channels_to_fields(msg: Any) -> dict[str, Any]:
    """Parse RC_CHANNELS fields into a channel dictionary."""
    channels: dict[int, int] = {}
    for channel_number in range(1, 19):
        value = _field(msg, f"chan{channel_number}_raw")
        if value is not None and int(value) > 0:
            channels[channel_number] = int(value)
    return {"rc_channels": channels}


def message_type(msg: Any) -> str | None:
    """Return the MAVLink message type name when available."""
    if hasattr(msg, "get_type"):
        return msg.get_type()
    msg_type = _field(msg, "type")
    if msg_type is None:
        return None
    return str(msg_type)


def _field(msg: Any, name: str) -> Any:
    return getattr(msg, name, None)
