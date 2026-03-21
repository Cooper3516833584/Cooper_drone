from __future__ import annotations

from pathlib import Path

from src.main import main

DATA = Path(__file__).parent / "data"


def test_main_dry_run_takeoff_mission() -> None:
    cfg_path = DATA / "cfg_dry_run.yaml"
    rc = main(["--config", str(cfg_path), "--dry-run", "--mission", "takeoff_and_hover"])
    assert rc == 0


def test_main_dry_run_default_standby() -> None:
    cfg_path = DATA / "cfg_dry_run.yaml"
    rc = main(["--config", str(cfg_path), "--dry-run"])
    assert rc == 0

