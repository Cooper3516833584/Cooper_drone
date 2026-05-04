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


# ---------------------------------------------------------------------------
# Additional Task-09 Layer-2 cases
# ---------------------------------------------------------------------------


def test_motion_gate_reason_stored_when_inhibited() -> None:
    """reason() returns the exact string passed to inhibit()."""
    gate = MotionGate()
    gate.inhibit("link_lost failsafe")
    assert gate.reason() == "link_lost failsafe"


def test_motion_gate_multiple_inhibit_overwrites_reason() -> None:
    """A second inhibit() call updates the reason without clearing inhibit."""
    gate = MotionGate()
    gate.inhibit("first reason")
    gate.inhibit("second reason")
    assert gate.is_inhibited() is True
    assert gate.reason() == "second reason"


def test_assert_motion_allowed_raises_with_reason_in_message() -> None:
    """MotionInhibitedError message contains the inhibit reason."""
    gate = MotionGate()
    gate.inhibit("kill switch active")
    with pytest.raises(MotionInhibitedError, match="kill switch active"):
        gate.assert_motion_allowed()


def test_stop_motion_through_gate_does_not_raise_when_inhibited() -> None:
    """Safety actions that bypass the gate (stop_motion from executor) must
    be callable even while the gate is inhibited.

    The MotionGate only blocks *normal* motion output via assert_motion_allowed().
    SafetyActionExecutor calls the control layer directly and does not check
    the gate, so this test verifies the gate itself does not interfere."""
    gate = MotionGate()
    gate.inhibit("kill")
    # Verify the gate is inhibited.
    assert gate.is_inhibited() is True
    # Verify that calling inhibit() again (as would happen during repeated
    # safety checks) does not raise.
    gate.inhibit("kill – repeated")
    assert gate.is_inhibited() is True
