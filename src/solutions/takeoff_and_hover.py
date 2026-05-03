"""High-level takeoff and hover solution."""

from __future__ import annotations

from src.solutions.common import SolutionContext, sleep_checked


def run_takeoff_and_hover(ctx: SolutionContext, *, altitude_m: float, hover_s: float) -> None:
    """Take off, hover for a fixed duration, and land."""
    ctx.logger.event("takeoff_hover_started", altitude_m=altitude_m, hover_s=hover_s)
    try:
        ctx.token.raise_if_cancelled()
        ctx.logger.event("takeoff_hover_set_guided_started")
        ctx.movement.set_mode("GUIDED")

        ctx.token.raise_if_cancelled()
        ctx.logger.event("takeoff_hover_arm_started")
        ctx.movement.arm()

        ctx.token.raise_if_cancelled()
        ctx.logger.event("takeoff_hover_takeoff_started", altitude_m=altitude_m)
        ctx.movement.takeoff(altitude_m)

        ctx.logger.event("takeoff_hover_hover_started", hover_s=hover_s)
        sleep_checked(ctx.token, hover_s)

        ctx.logger.event("takeoff_hover_land_started")
        ctx.movement.land()
        ctx.logger.event("takeoff_hover_finished", status="ok")
    except Exception as exc:
        ctx.logger.event("takeoff_hover_interrupted", error=repr(exc), recovery="land_then_stop_motion")
        _recover_with_land_or_stop(ctx)
        raise


def _recover_with_land_or_stop(ctx: SolutionContext) -> None:
    try:
        ctx.movement.land()
        ctx.logger.event("takeoff_hover_recovery_land_sent")
    except Exception as land_error:
        ctx.logger.event("takeoff_hover_recovery_land_failed", error=repr(land_error))
        ctx.movement.stop_motion()
        ctx.logger.event("takeoff_hover_recovery_stop_motion_sent")
