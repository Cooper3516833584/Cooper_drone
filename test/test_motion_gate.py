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
        gate.assert_allowed()


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
