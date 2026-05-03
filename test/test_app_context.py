"""Tests for application context assembly."""

from __future__ import annotations

from pathlib import Path

from src.app_context import build_app_context


def test_app_context_close_is_idempotent(dry_run_config_path: Path) -> None:
    """Allow AppContext.close to be called more than once."""
    ctx = build_app_context(str(dry_run_config_path), dry_run=True)

    ctx.close()
    ctx.close()

    assert ctx.safety is None
    assert ctx.connection is None
