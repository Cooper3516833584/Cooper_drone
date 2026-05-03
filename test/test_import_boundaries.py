"""Import and source boundary tests."""

from __future__ import annotations

from pathlib import Path


ROOT_TASK_FORBIDDEN = ["pymavlink", "mavutil", "command_long_send", "recv_match"]
SOLUTIONS_FORBIDDEN = ["pymavlink", "mavutil", "command_long_send", "recv_match"]
VISION_FORBIDDEN = ["pymavlink", "mavutil", "DroneMovement", "MavlinkCommandService"]
PYMAVLINK_ALLOWED = {
    Path("src/mavlink_connection.py"),
    Path("src/mavlink_commands.py"),
    Path("src/mavlink_messages.py"),
    Path("src/movement.py"),
}


def test_root_tasks_do_not_use_low_level_mavlink() -> None:
    """Keep root task scripts free of low-level MAVLink calls."""
    for path in Path(".").glob("task_*.py"):
        _assert_forbidden_terms_absent(path, ROOT_TASK_FORBIDDEN)


def test_solutions_do_not_use_low_level_mavlink() -> None:
    """Keep solutions free of low-level MAVLink calls."""
    for path in Path("src/solutions").glob("*.py"):
        _assert_forbidden_terms_absent(path, SOLUTIONS_FORBIDDEN)


def test_vision_does_not_depend_on_control_or_mavlink() -> None:
    """Keep Vision independent from control and MAVLink layers."""
    for path in Path("src/Vision").glob("*.py"):
        _assert_forbidden_terms_absent(path, VISION_FORBIDDEN)


def test_only_low_level_files_may_import_pymavlink() -> None:
    """Restrict pymavlink usage to low-level MAVLink modules."""
    offenders = []
    for path in Path("src").rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        if "pymavlink" in source and path not in PYMAVLINK_ALLOWED:
            offenders.append(str(path))

    assert offenders == []


def _assert_forbidden_terms_absent(path: Path, terms: list[str]) -> None:
    source = path.read_text(encoding="utf-8")
    for term in terms:
        assert term not in source, f"{term} found in {path}"
