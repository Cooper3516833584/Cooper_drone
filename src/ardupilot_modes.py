"""ArduPilot mode mappings used at MAVLink boundaries."""

from __future__ import annotations


COPTER_MODE_BY_CUSTOM_MODE = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}


def copter_mode_name(custom_mode: int) -> str | None:
    """Return the ArduPilot Copter mode name for a custom_mode number."""
    try:
        return COPTER_MODE_BY_CUSTOM_MODE.get(int(custom_mode))
    except (TypeError, ValueError):
        return None
