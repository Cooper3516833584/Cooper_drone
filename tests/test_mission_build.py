from __future__ import annotations

import logging
from pathlib import Path

import pytest

from src.config_loader import load_config
from src.fc_link import FlightControllerLink
from src.mav_state import VehicleStateCache
from src.mission_base import CancelToken, MissionContext, build_mission
from src.safety import SafetyManager
from src.telemetry import TelemetryHub

DATA = Path(__file__).parent / "data"


def _ctx() -> MissionContext:
    cfg = load_config(DATA / "cfg_dry_run.yaml")
    cache = VehicleStateCache()
    link = FlightControllerLink(cfg.mavlink, cache, dry_run=True, logger=logging.getLogger("test.link"))
    link.connect()
    telemetry = TelemetryHub(cache, sample_hz=cfg.telemetry.sample_hz, logger=logging.getLogger("test.telemetry"))
    token = CancelToken()
    safety = SafetyManager(link.get_session(), cfg, token, logger=logging.getLogger("test.safety"))
    return MissionContext(
        cfg=cfg,
        link=link,
        safety=safety,
        telemetry=telemetry,
        logger=logging.getLogger("test.mission"),
        cancel=token,
    )


def test_build_takeoff_and_placeholder_missions() -> None:
    ctx = _ctx()
    m_takeoff = build_mission("takeoff_and_hover", ctx)
    m_square = build_mission("waypoint_square", ctx)
    m_vision = build_mission("vision_track", ctx)
    assert m_takeoff.name == "takeoff_and_hover"
    assert m_square.name == "waypoint_square"
    assert m_vision.name == "vision_track"

    with pytest.raises(NotImplementedError):
        m_square.run(ctx, ctx.cancel)
    with pytest.raises(NotImplementedError):
        m_vision.run(ctx, ctx.cancel)


def test_build_unknown_raises() -> None:
    with pytest.raises(ValueError):
        build_mission("unknown_task", _ctx())

