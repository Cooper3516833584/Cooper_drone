"""Compatibility helpers for legacy motion inhibit calls."""

from __future__ import annotations

import threading

from src.motion_gate import MotionGate


_DEFAULT_MOTION_GATE = MotionGate()
_DEFAULT_GATE_LOCK = threading.RLock()


def get_default_motion_gate() -> MotionGate:
    """Return the default compatibility motion gate."""
    with _DEFAULT_GATE_LOCK:
        return _DEFAULT_MOTION_GATE


def set_default_motion_gate(gate: MotionGate) -> None:
    """Replace the default compatibility motion gate for tests or legacy sessions."""
    global _DEFAULT_MOTION_GATE
    with _DEFAULT_GATE_LOCK:
        _DEFAULT_MOTION_GATE = gate


def inhibit_motion_outputs(reason: str = "") -> None:
    """Compatibility wrapper that inhibits the default motion gate."""
    get_default_motion_gate().inhibit(reason)


def clear_motion_inhibit() -> None:
    """Compatibility wrapper that clears the default motion gate."""
    get_default_motion_gate().clear()


def motion_output_inhibited() -> bool:
    """Return whether the default compatibility gate is inhibited."""
    return get_default_motion_gate().is_inhibited()
