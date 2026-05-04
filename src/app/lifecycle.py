"""Application lifecycle management including safe shutdown.

The safe_exit function is the single entry point for application shutdown.
It centralises all safety actions so that no other code calls control.* or
DroneMovement methods directly during exit.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.safety_actions import SafetyActionExecutor

if TYPE_CHECKING:
    from src.app_context import AppContext


def safe_exit(
    ctx: AppContext,
    *,
    mission_was_running: bool = False,
    cancel_reason: str | None = None,
) -> None:
    """Perform safe shutdown actions, then close runtime resources.

    Shutdown strategy
    -----------------
    * If *mission_was_running* is True: cancel the token, stop normal motion
      output, then execute the configured ``exit_action`` (default ``land``).
    * If *mission_was_running* is False (standby / mission=none): execute
      ``standby_exit_action`` (default ``loiter``).
    * When ``dry_run`` is True the connection does not exist -- all flight
      actions are skipped.
    * Every action goes through :class:`SafetyActionExecutor` so results are
      logged and failures are recorded.
    * No individual action failure will prevent the remaining cleanup steps
      (stop supervisor, close connection, close logger) from running.

    Parameters
    ----------
    ctx:
        The fully assembled application context.
    mission_was_running:
        Set to ``True`` when a flight mission armed / entered RUNNING state.
    cancel_reason:
        Human-readable reason passed to ``CancellationToken.cancel()``.
        Only used when *mission_was_running* is True.
    """
    dry_run = ctx.cfg.dry_run

    # 1. Cancel the mission token so any concurrent tasks stop.
    if mission_was_running and cancel_reason is not None:
        ctx.token.cancel(cancel_reason)
        ctx.logger.event("safe_exit_mission_cancelled", reason=cancel_reason)

    # 2. Choose the appropriate exit action.
    exit_action: str
    if mission_was_running:
        exit_action = ctx.cfg.safety.exit_action
    else:
        exit_action = ctx.cfg.safety.standby_exit_action

    # 3. Execute flight actions through the safety executor.
    if not dry_run:
        executor = SafetyActionExecutor(ctx.movement, ctx.logger)
        executor._current_state = "shutdown"

        # Always attempt stop_motion first so normal velocity output ceases.
        _execute_action(executor, ctx, "stop", reason="safe_exit: stop motion")

        # Then the configured exit action.
        if exit_action and exit_action != "none":
            _execute_action(executor, ctx, exit_action, reason=f"safe_exit: {exit_action}")
    else:
        ctx.logger.event("safe_exit_dry_run_skipped", exit_action=exit_action)

    # 4. Stop the safety supervisor.
    if ctx.safety is not None:
        try:
            ctx.safety.stop()
        except Exception:
            ctx.logger.app_logger.exception("safe_exit: safety supervisor stop failed")
        ctx.safety = None

    # 5. Close the MAVLink connection.
    if ctx.connection is not None:
        try:
            ctx.connection.close()
        except Exception:
            ctx.logger.app_logger.exception("safe_exit: connection close failed")
        ctx.connection = None

    # 6. Close all log files.
    try:
        ctx.logger.close()
    except Exception:
        # Last resort -- log to stderr since app_logger may already be closed.
        import sys
        print("safe_exit: logger close failed", file=sys.stderr)


def _execute_action(
    executor: SafetyActionExecutor,
    ctx: AppContext,
    action: str,
    *,
    reason: str,
) -> None:
    """Execute one safety action, logging but never raising."""
    try:
        result = executor.execute(action, reason=reason)
        ctx.logger.safety(
            "safe_exit_action",
            action=result.action,
            ok=result.ok,
            error=result.error,
        )
    except Exception:
        ctx.logger.app_logger.exception(
            "safe_exit: action %s failed with unhandled exception", action
        )
