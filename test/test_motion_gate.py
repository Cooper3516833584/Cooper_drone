"""Tests for the motion gate."""

from __future__ import annotations

import threading

import pytest

from src.motion_gate import MotionGate, MotionInhibitedError


def test_motion_gate_blocks_when_inhibited() -> None:
    """Raise when motion is inhibited."""
    gate = MotionGate()
    gate.inhibit("test inhibit")

    with pytest.raises(MotionInhibitedError, match="test inhibit"):
        gate.assert_motion_allowed()


def test_motion_gate_defaults_to_allowed() -> None:
    """A new gate allows normal motion output."""
    gate = MotionGate()

    assert gate.is_inhibited() is False
    assert gate.reason() == ""
    gate.assert_motion_allowed()


def test_motion_gate_clear_restores_allowed_state() -> None:
    """Clear removes inhibit state and reason."""
    gate = MotionGate()
    gate.inhibit("operator")

    assert gate.reason() == "operator"
    gate.clear()

    assert gate.is_inhibited() is False
    assert gate.reason() == ""
    gate.assert_motion_allowed()


def test_motion_gate_instances_do_not_share_state() -> None:
    """Separate gates do not pollute each other."""
    first = MotionGate()
    second = MotionGate()

    first.inhibit("first")

    assert first.is_inhibited() is True
    assert first.reason() == "first"
    assert second.is_inhibited() is False
    second.assert_motion_allowed()


def test_motion_gate_thread_safe_basic() -> None:
    """Exercise concurrent inhibit and clear operations."""
    gate = MotionGate()

    def worker(index: int) -> None:
        for _ in range(100):
            gate.inhibit(f"reason-{index}")
            assert gate.is_inhibited()
            gate.clear()

    threads = [threading.Thread(target=worker, args=(index,)) for index in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=1.0)

    assert all(not thread.is_alive() for thread in threads)
    gate.clear()
    gate.assert_allowed()
