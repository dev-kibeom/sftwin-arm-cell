"""In-process bridge between the Hub and active Material Supply Adapter."""

from __future__ import annotations

from typing import Any

_adapter: Any | None = None


def bind_adapter(adapter: Any | None) -> None:
    global _adapter
    _adapter = adapter


def load_demonstration(scenario_id: str) -> int:
    if _adapter is None:
        raise RuntimeError("Isaac Material Supply is unavailable")
    return _adapter.load_demonstration(scenario_id)


def demonstration_loaded() -> bool:
    return bool(_adapter is not None and _adapter._demonstration_loaded)


def load_unavailable_reason() -> str | None:
    if _adapter is None:
        return "Isaac Material Supply Adapter is unavailable"
    return _adapter.demonstration_load_unavailable_reason()
