"""Tests for MAVLink command service."""

from __future__ import annotations

import json
from pathlib import Path

from src.config_loader import MavlinkConfig
from src.logging_runtime import RuntimeLogger
from src.mavlink_commands import (
    MAV_RESULT_ACCEPTED,
    MavlinkCommandRejected,
    MavlinkCommandService,
    MavlinkCommandTimeout,
)
from src.mavlink_connection import MavlinkConnection
from test.fake_mavlink import FakeMavlinkConnection, FakeMavlinkMessage


def test_dry_run_command_long_returns_accepted(runtime_logger) -> None:
    """Return a simulated accepted ACK during dry-run."""
    try:
        connection = MavlinkConnection(_mavlink_config("dry-run"), runtime_logger)
        service = MavlinkCommandService(connection, runtime_logger)

        ack = service.command_long(400, [1.0], name="test_arm")

        assert ack.command == 400
        assert ack.result == MAV_RESULT_ACCEPTED
        assert ack.result_name == "ACCEPTED"
    finally:
        runtime_logger.close()


def test_fake_ack_accepted_succeeds(runtime_logger) -> None:
    """Send COMMAND_LONG and accept a matching fake ACK."""
    fake_connection = FakeMavlinkConnection()
    try:
        connection = _connected_wrapper(fake_connection, runtime_logger)
        service = MavlinkCommandService(connection, runtime_logger)
        connection.start_receiver()

        ack = service.command_long(176, [4.0], timeout_s=0.5, retries=0, name="set_mode")

        assert ack.result_name == "ACCEPTED"
        assert fake_connection.mav.command_long_calls[0]["command"] == 176
    finally:
        connection.close()
        runtime_logger.close()


def test_fake_ack_rejected_raises(runtime_logger) -> None:
    """Raise an error when COMMAND_ACK rejects the command."""
    fake_connection = FakeMavlinkConnection()
    fake_connection.mav.add_ack_result(2)
    try:
        connection = _connected_wrapper(fake_connection, runtime_logger)
        service = MavlinkCommandService(connection, runtime_logger)
        connection.start_receiver()

        try:
            service.command_long(400, [1.0], timeout_s=0.5, retries=0, name="arm")
        except MavlinkCommandRejected as exc:
            assert "DENIED" in str(exc)
        else:
            raise AssertionError("Expected MavlinkCommandRejected")
    finally:
        connection.close()
        runtime_logger.close()


def test_no_ack_retries_and_times_out(runtime_logger) -> None:
    """Retry COMMAND_LONG and raise timeout when no ACK arrives."""
    fake_connection = FakeMavlinkConnection()
    fake_connection.mav.auto_ack = False
    try:
        connection = _connected_wrapper(fake_connection, runtime_logger)
        service = MavlinkCommandService(connection, runtime_logger)
        connection.start_receiver()

        try:
            service.command_long(300, timeout_s=0.02, retries=2, name="timeout_test")
        except MavlinkCommandTimeout:
            pass
        else:
            raise AssertionError("Expected MavlinkCommandTimeout")

        assert len(fake_connection.mav.command_long_calls) == 3
    finally:
        connection.close()
        runtime_logger.close()


def test_commands_jsonl_contains_command_result_and_duration(runtime_logger) -> None:
    """Log command name, params, result, and duration."""
    try:
        connection = MavlinkConnection(_mavlink_config("dry-run"), runtime_logger)
        service = MavlinkCommandService(connection, runtime_logger)
        service.command_long(400, [1.0, 2.0], name="logged_command")
    finally:
        runtime_logger.close()

    records = _read_jsonl(runtime_logger.run_dir / "commands.jsonl")
    record = records[-1]
    assert record["name"] == "logged_command"
    assert record["params"][:2] == [1.0, 2.0]
    assert record["result_name"] == "ACCEPTED"
    assert "duration_ms" in record


def test_unknown_ack_does_not_raise(runtime_logger) -> None:
    """Log unknown ACKs without raising from the receiver thread."""
    fake_connection = FakeMavlinkConnection([FakeMavlinkMessage("COMMAND_ACK", command=999, result=0)])
    try:
        connection = _connected_wrapper(fake_connection, runtime_logger)
        MavlinkCommandService(connection, runtime_logger)
        connection.start_receiver()
        connection.stop_receiver()
    finally:
        connection.close()
        runtime_logger.close()

    event_text = (runtime_logger.run_dir / "events.jsonl").read_text(encoding="utf-8")
    assert "unknown_command_ack" in event_text


def _connected_wrapper(fake_connection: FakeMavlinkConnection, runtime_logger: RuntimeLogger) -> MavlinkConnection:
    connection = MavlinkConnection(
        _mavlink_config("udp:127.0.0.1:14550"),
        runtime_logger,
        lambda *args, **kwargs: fake_connection,
    )
    connection.connect()
    return connection


def _mavlink_config(connection_string: str) -> MavlinkConfig:
    return MavlinkConfig(
        connection_string=connection_string,
        baud=115200,
        source_system=255,
        source_component=0,
        target_system=None,
        target_component=None,
        heartbeat_timeout_s=1.0,
        command_timeout_s=0.1,
        command_retries=1,
    )


def _read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
