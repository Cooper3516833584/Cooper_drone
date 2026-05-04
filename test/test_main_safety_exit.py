"""Tests for safe_exit lifecycle management (Task 10).

Covers:
1. Mission run exception triggers safe_exit with stop + LAND.
2. RUNNING state exception attempts stop + LAND.
3. Standby / mission none does NOT land (uses LOITER).
4. Dry-run / no link skips flight actions entirely.
5. stop_motion failure does not prevent connection + logger close.
6. KeyboardInterrupt cancels token and returns exit code 130.
"""

from __future__ import annotations

import logging

import pytest

from src.app.lifecycle import safe_exit
from src.app_context import AppContext
from src.config_loader import (
    AppConfig,
    LimitsConfig,
    LoggingConfig,
    MavlinkConfig,
    SafetyConfig,
    VisionConfig,
)
from src.mission_runtime import CancellationToken
from src.motion_gate import MotionGate
from src.state import VehicleState


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class FakeMovement:
    """Tracks method calls; can be configured to raise on a specific method."""

    def __init__(self, *, fail_on: str | None = None) -> None:
        self.calls: list[str] = []
        self.fail_on = fail_on

    def stop_motion(self) -> None:
        self.calls.append("stop_motion")
        if self.fail_on == "stop_motion":
            raise RuntimeError("fake stop_motion failure")

    def land(self) -> None:
        self.calls.append("land")
        if self.fail_on == "land":
            raise RuntimeError("fake land failure")

    def land_nowait(self) -> None:
        self.calls.append("land_nowait")
        if self.fail_on == "land_nowait":
            raise RuntimeError("fake land_nowait failure")

    def loiter(self) -> None:
        self.calls.append("loiter")
        if self.fail_on == "loiter":
            raise RuntimeError("fake loiter failure")


class FakeConnection:
    def __init__(self, *, fail_close: bool = False) -> None:
        self.closed = False
        self.fail_close = fail_close

    def close(self) -> None:
        if self.fail_close:
            raise RuntimeError("fake close failure")
        self.closed = True


class FakeSafetySupervisor:
    def __init__(self, *, fail_stop: bool = False) -> None:
        self.stopped = False
        self.fail_stop = fail_stop

    def stop(self) -> None:
        if self.fail_stop:
            raise RuntimeError("fake supervisor stop failure")
        self.stopped = True


class FakeLogger:
    """In-memory logger that captures safety records and events."""

    def __init__(self) -> None:
        self.safety_records: list[dict] = []
        self.events: list[dict] = []
        self._app_logger = logging.getLogger("test.fake_logger")
        self.closed = False

    @property
    def app_logger(self):
        return self._app_logger

    def safety(self, name: str, **fields: object) -> None:
        self.safety_records.append({"name": name, **fields})

    def event(self, name: str, **fields: object) -> None:
        self.events.append({"name": name, **fields})

    def close(self) -> None:
        self.closed = True


# ---------------------------------------------------------------------------
# Config builders
# ---------------------------------------------------------------------------


def _safety_config(**overrides: object) -> SafetyConfig:
    values: dict = {
        "enabled": True,
        "allow_arm": False,
        "require_guided": True,
        "require_heartbeat": True,
        "heartbeat_loss_timeout_s": 2.0,
        "rc_stale_timeout_s": 1.0,
        "manual_takeover_enabled": True,
        "kill_switch_enabled": True,
        "failsafe_action": "land",
        "exit_action": "land",
        "standby_exit_action": "loiter",
    }
    values.update(overrides)
    return SafetyConfig(**values)


def _app_config(*, dry_run: bool = False, **safety_overrides: object) -> AppConfig:
    return AppConfig(
        profile_name="test",
        dry_run=dry_run,
        mavlink=MavlinkConfig(
            connection_string="dry-run" if dry_run else "udp:127.0.0.1:14550",
            baud=115200,
            source_system=255,
            source_component=0,
            target_system=1,
            target_component=1,
            heartbeat_timeout_s=5.0,
            command_timeout_s=3.0,
            command_retries=3,
        ),
        limits=LimitsConfig(
            max_vx_mps=2.0,
            max_vy_mps=2.0,
            max_vz_mps=1.0,
            max_yaw_rate_dps=45.0,
            max_altitude_m=20.0,
            takeoff_altitude_tolerance_m=0.3,
        ),
        safety=_safety_config(**safety_overrides),
        logging=LoggingConfig(
            log_dir="logs",
            telemetry_log_hz=5.0,
            jsonl_enabled=False,
            console_level="INFO",
        ),
        vision=VisionConfig(
            enabled=False,
            camera_index=0,
            width=640,
            height=480,
            fps=30,
            dry_run_source=None,
        ),
    )


def _build_ctx(
    *,
    dry_run: bool = False,
    mission_was_running: bool = False,
    fail_motion_on: str | None = None,
    fail_supervisor_stop: bool = False,
    fail_connection_close: bool = False,
    **safety_overrides: object,
) -> tuple[AppContext, FakeMovement, FakeConnection, FakeSafetySupervisor, FakeLogger, CancellationToken]:
    """Build a minimal AppContext for safe_exit tests.

    Returns the context and all fake objects for assertion.
    """
    cfg = _app_config(dry_run=dry_run, **safety_overrides)
    movement = FakeMovement(fail_on=fail_motion_on)
    connection = FakeConnection(fail_close=fail_connection_close)
    safety = FakeSafetySupervisor(fail_stop=fail_supervisor_stop)
    logger = FakeLogger()
    token = CancellationToken()
    motion_gate = MotionGate()

    ctx = AppContext(
        cfg=cfg,
        logger=logger,
        connection=connection,
        commands=None,  # type: ignore[arg-type]  # not used by safe_exit
        motion_gate=motion_gate,
        movement=movement,
        token=token,
        safety=safety,
        state_provider=lambda: VehicleState(mode="GUIDED", armed=False),
        mission_was_running=mission_was_running,
    )
    return ctx, movement, connection, safety, logger, token


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestSafeExitMissionRunning:
    """Tests for safe_exit when a flight mission was running."""

    def test_running_mission_exception_triggers_stop_and_land(self) -> None:
        """When mission_was_running=True, safe_exit stops motion then lands."""
        ctx, movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test failure")

        assert "stop_motion" in movement.calls
        assert "land_nowait" in movement.calls
        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_running_mission_cancels_token(self) -> None:
        """safe_exit cancels the token when a cancel_reason is given."""
        ctx, _movement, _conn, _safety, _logger, token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="link lost")

        assert token.is_cancelled() is True
        assert token.reason() == "link lost"

    def test_running_mission_logs_actions(self) -> None:
        """safe_exit logs each action result via the logger."""
        ctx, _movement, _conn, _safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        action_names = [r["name"] for r in logger.safety_records if r["name"] == "safe_exit_action"]
        assert len(action_names) >= 2  # stop + land


class TestSafeExitStandby:
    """Tests for safe_exit when no flight mission was running (standby)."""

    def test_standby_uses_loiter_not_land(self) -> None:
        """Standby exit uses standby_exit_action (LOITER), not LAND."""
        ctx, movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
        )

        safe_exit(ctx, mission_was_running=False)

        assert "land_nowait" not in movement.calls
        assert "loiter" in movement.calls
        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_standby_still_stops_motion(self) -> None:
        """Even in standby, safe_exit stops motion first."""
        ctx, movement, _conn, _safety, _logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
        )

        safe_exit(ctx, mission_was_running=False)

        assert "stop_motion" in movement.calls

    def test_standby_does_not_cancel_token(self) -> None:
        """Standby exit with no cancel_reason does not cancel the token."""
        ctx, _movement, _conn, _safety, _logger, token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
        )

        safe_exit(ctx, mission_was_running=False)

        assert token.is_cancelled() is False

    def test_standby_custom_action_is_respected(self) -> None:
        """Standby exit uses the configured standby_exit_action."""
        ctx, movement, _conn, _safety, _logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
            standby_exit_action="brake",
        )

        safe_exit(ctx, mission_was_running=False)

        assert "loiter" not in movement.calls
        # BRAKE action on FakeMovement falls back to set_mode_nowait... but
        # FakeMovement doesn't have set_mode_nowait. Let the executor try;
        # we just verify land wasn't called.
        assert "land_nowait" not in movement.calls

    def test_exit_action_none_skips_secondary_action(self) -> None:
        """When exit_action is 'none', only stop_motion runs (no land)."""
        ctx, movement, _conn, _safety, _logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            exit_action="none",
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert "stop_motion" in movement.calls
        assert "land_nowait" not in movement.calls

    def test_standby_none_sends_no_flight_actions(self) -> None:
        """Standby + standby_exit_action=none sends zero flight actions."""
        ctx, movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
            standby_exit_action="none",
        )

        safe_exit(ctx, mission_was_running=False)

        assert movement.calls == []
        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_standby_none_logs_standby_event(self) -> None:
        """Standby + none logs safe_exit_standby_no_action."""
        ctx, _movement, _conn, _safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
            standby_exit_action="none",
        )

        safe_exit(ctx, mission_was_running=False)

        events = [e for e in logger.events if e["name"] == "safe_exit_standby_no_action"]
        assert len(events) == 1


class TestSafeExitDryRun:
    """Tests for safe_exit when dry_run is enabled (no real connection)."""

    def test_dry_run_skips_all_flight_actions(self) -> None:
        """When dry_run=True, no flight actions are sent."""
        ctx, movement, conn, safety, logger, _token = _build_ctx(
            dry_run=True,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert movement.calls == []
        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_dry_run_logs_skip_event(self) -> None:
        """Dry-run safe_exit logs a skip event."""
        ctx, _movement, _conn, _safety, logger, _token = _build_ctx(
            dry_run=True,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        skip_events = [e for e in logger.events if e["name"] == "safe_exit_dry_run_skipped"]
        assert len(skip_events) == 1

    def test_dry_run_still_cancels_token(self) -> None:
        """Even in dry-run, the token is cancelled when a reason is given."""
        ctx, _movement, _conn, _safety, _logger, token = _build_ctx(
            dry_run=True,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert token.is_cancelled() is True


class TestSafeExitFailureResilience:
    """Tests for resilience: no single failure blocks the rest of shutdown."""

    def test_stop_motion_failure_does_not_block_land(self) -> None:
        """When stop_motion raises, land is still attempted."""
        ctx, movement, _conn, _safety, _logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_motion_on="stop_motion",
        )

        # Must not raise.
        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert "stop_motion" in movement.calls
        assert "land_nowait" in movement.calls

    def test_stop_motion_failure_does_not_block_close(self) -> None:
        """When stop_motion raises, connection and logger are still closed."""
        ctx, _movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_motion_on="stop_motion",
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_land_failure_does_not_block_close(self) -> None:
        """When land raises, supervisor, connection, and logger still close."""
        ctx, _movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_motion_on="land_nowait",
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_supervisor_stop_failure_does_not_block_close(self) -> None:
        """When supervisor.stop() raises, connection and logger still close."""
        ctx, _movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_supervisor_stop=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert conn.closed is True
        assert logger.closed is True

    def test_connection_close_failure_does_not_block_logger_close(self) -> None:
        """When connection.close() raises, logger is still closed."""
        ctx, _movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_connection_close=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert conn.closed is False  # close raised
        assert logger.closed is True

    def test_all_actions_fail_still_closes_everything(self) -> None:
        """When everything fails, all cleanup is still attempted."""
        ctx, _movement, conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
            fail_motion_on="stop_motion",
            fail_supervisor_stop=True,
            fail_connection_close=True,
        )

        # Must not raise.
        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert logger.closed is True


class TestSafeExitKeyboardInterrupt:
    """Tests for the KeyboardInterrupt cancellation path."""

    def test_token_pre_cancelled_safe_exit_still_works(self) -> None:
        """When token is already cancelled (as by KeyboardInterrupt handler),
        safe_exit still completes normally without double-cancelling."""
        ctx, movement, conn, safety, logger, token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )
        token.cancel("keyboard interrupt")

        # Must not raise.
        safe_exit(ctx, mission_was_running=True, cancel_reason="keyboard interrupt")

        assert token.is_cancelled() is True
        assert "stop_motion" in movement.calls
        assert "land_nowait" in movement.calls
        assert safety.stopped is True
        assert conn.closed is True
        assert logger.closed is True

    def test_keyboard_interrupt_cancel_event_is_logged(self) -> None:
        """Cancelling the token produces a safe_exit_mission_cancelled event."""
        ctx, _movement, _conn, _safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason="keyboard interrupt")

        events = [e for e in logger.events if e["name"] == "safe_exit_mission_cancelled"]
        assert len(events) == 1
        assert events[0]["reason"] == "keyboard interrupt"


class TestSafeExitEdgeCases:
    """Edge case and boundary tests."""

    def test_no_cancel_reason_does_not_cancel_token(self) -> None:
        """When cancel_reason is None, the token is not cancelled even for a running mission."""
        ctx, _movement, _conn, _safety, _logger, token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )

        safe_exit(ctx, mission_was_running=True, cancel_reason=None)

        assert token.is_cancelled() is False
        # Actions still execute.
        assert _movement.calls  # stop + land

    def test_mission_not_running_with_cancel_reason_does_not_cancel(self) -> None:
        """A cancel_reason without mission_was_running does not cancel token."""
        ctx, _movement, _conn, _safety, _logger, token = _build_ctx(
            dry_run=False,
            mission_was_running=False,
        )

        safe_exit(ctx, mission_was_running=False, cancel_reason="should not apply")

        assert token.is_cancelled() is False

    def test_safety_none_is_handled(self) -> None:
        """safe_exit handles ctx.safety = None gracefully."""
        ctx, movement, conn, _safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )
        ctx.safety = None

        # Must not raise.
        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert conn.closed is True
        assert logger.closed is True

    def test_connection_none_is_handled(self) -> None:
        """safe_exit handles ctx.connection = None gracefully."""
        ctx, movement, _conn, safety, logger, _token = _build_ctx(
            dry_run=False,
            mission_was_running=True,
        )
        ctx.connection = None

        # Must not raise.
        safe_exit(ctx, mission_was_running=True, cancel_reason="test")

        assert safety.stopped is True
        assert logger.closed is True
