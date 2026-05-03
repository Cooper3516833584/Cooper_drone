"""High-level placeholder for vision follow missions."""

from __future__ import annotations

from src.setpoint_streamer import BodyVelocityStreamer
from src.solutions.common import SolutionContext, sleep_checked


def run_vision_follow(ctx: SolutionContext, *, altitude_m: float, duration_s: float) -> None:
    """Run a placeholder vision follow mission with zero velocity setpoints."""
    ctx.logger.event("vision_follow_started", altitude_m=altitude_m, duration_s=duration_s)
    streamer = BodyVelocityStreamer(ctx.movement, ctx.logger, rate_hz=10.0, setpoint_ttl_s=0.5)
    streamer_started = False
    try:
        ctx.token.raise_if_cancelled()
        ctx.movement.set_mode("GUIDED")
        ctx.token.raise_if_cancelled()
        ctx.movement.arm()
        ctx.token.raise_if_cancelled()
        ctx.movement.takeoff(altitude_m)

        streamer.start()
        streamer_started = True
        streamer.update(0.0, 0.0, 0.0, 0.0)
        sleep_checked(ctx.token, duration_s)
        streamer.stop()
        streamer_started = False

        ctx.movement.land()
        ctx.logger.event("vision_follow_finished", status="ok")
    except Exception as exc:
        ctx.logger.event("vision_follow_interrupted", error=repr(exc), recovery="stop_streamer_land_then_stop_motion")
        if streamer_started:
            streamer.stop()
        _recover_with_land_or_stop(ctx)
        raise


def _recover_with_land_or_stop(ctx: SolutionContext) -> None:
    try:
        ctx.movement.land()
        ctx.logger.event("vision_follow_recovery_land_sent")
    except Exception as land_error:
        ctx.logger.event("vision_follow_recovery_land_failed", error=repr(land_error))
        ctx.movement.stop_motion()
        ctx.logger.event("vision_follow_recovery_stop_motion_sent")
