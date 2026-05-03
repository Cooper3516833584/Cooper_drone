"""High-level square movement solution using body velocity estimates."""

from __future__ import annotations

from src.solutions.common import SolutionContext, sleep_checked


def run_waypoint_square(
    ctx: SolutionContext,
    *,
    altitude_m: float,
    side_m: float,
    speed_mps: float,
) -> None:
    """Fly a square using timed body velocity segments."""
    if side_m <= 0:
        raise ValueError("side_m must be greater than 0")
    if speed_mps <= 0:
        raise ValueError("speed_mps must be greater than 0")

    ctx.logger.event("waypoint_square_started", altitude_m=altitude_m, side_m=side_m, speed_mps=speed_mps)
    try:
        ctx.token.raise_if_cancelled()
        ctx.movement.set_mode("GUIDED")
        ctx.token.raise_if_cancelled()
        ctx.movement.arm()
        ctx.token.raise_if_cancelled()
        ctx.movement.takeoff(altitude_m)

        duration_s = side_m / speed_mps
        _run_segment(ctx, "forward", speed_mps, 0.0, 0.0, duration_s)
        _run_segment(ctx, "right", 0.0, speed_mps, 0.0, duration_s)
        _run_segment(ctx, "backward", -speed_mps, 0.0, 0.0, duration_s)
        _run_segment(ctx, "left", 0.0, -speed_mps, 0.0, duration_s)

        ctx.logger.event("waypoint_square_land_started")
        ctx.movement.land()
        ctx.logger.event("waypoint_square_finished", status="ok")
    except Exception as exc:
        ctx.logger.event("waypoint_square_interrupted", error=repr(exc), recovery="land_then_stop_motion")
        _recover_with_land_or_stop(ctx)
        raise


def _run_segment(
    ctx: SolutionContext,
    name: str,
    vx_mps: float,
    vy_mps: float,
    vz_mps: float,
    duration_s: float,
) -> None:
    ctx.token.raise_if_cancelled()
    ctx.logger.event("waypoint_square_segment_started", segment=name, duration_s=duration_s)
    try:
        ctx.movement.send_body_velocity(vx_mps, vy_mps, vz_mps)
        sleep_checked(ctx.token, duration_s)
    finally:
        ctx.movement.stop_motion()
        ctx.logger.event("waypoint_square_segment_finished", segment=name)


def _recover_with_land_or_stop(ctx: SolutionContext) -> None:
    try:
        ctx.movement.land()
        ctx.logger.event("waypoint_square_recovery_land_sent")
    except Exception as land_error:
        ctx.logger.event("waypoint_square_recovery_land_failed", error=repr(land_error))
        ctx.movement.stop_motion()
        ctx.logger.event("waypoint_square_recovery_stop_motion_sent")
