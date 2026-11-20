"""Cell-owned, in-process supply capability consumed by the common UI Hub.

Acceptance is only the Cell adapter's response to the request; it is never
material readiness, delivery completion, or permission to start a mission.
"""

from copy import deepcopy
from typing import Callable


_port = None


def register_supply_port(owner, descriptor: dict, request: Callable, status: Callable, *, set_automatic=None):
    """Bind the active Cell. Descriptor contains materials (id/label), count_label,
    and default_count. Callbacks take (material_id, count) and no args respectively.
    """
    global _port
    if owner is None or not callable(request) or not callable(status):
        raise ValueError("Supply requires an owner and request/status callbacks")
    materials = descriptor.get("materials", [])
    if not materials or any(not material.get("id") for material in materials):
        raise ValueError("Supply descriptor requires materials with ids")
    descriptor = deepcopy(descriptor)
    if set_automatic is not None:
        descriptor["automatic_control"] = True
    _port = (owner, descriptor, request, status, set_automatic)


def clear_supply_port(owner):
    """Only the registered owner may clear its port during teardown."""
    global _port
    if _port is not None and _port[0] is owner:
        _port = None


def supply_descriptor():
    return deepcopy(_port[1]) if _port is not None else None


def supply_status():
    if _port is None:
        return {"available": False, "detail": "Cell supply capability unavailable"}
    try:
        return dict(_port[3]())
    except (RuntimeError, ValueError, KeyError, TypeError, AttributeError) as error:
        return {"available": False, "detail": str(error)}


def request_supply(material_id: str, count: int):
    port = _port
    if port is None:
        return {"accepted": False, "detail": "Cell supply capability unavailable"}
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        return {"accepted": False, "detail": "Supply count must be a positive integer"}
    if material_id not in {material["id"] for material in port[1]["materials"]}:
        return {"accepted": False, "detail": "Unknown supply material"}
    try:
        result = dict(port[2](material_id, count))
        return {**result, "accepted": result.get("accepted") is True,
                "detail": str(result.get("detail", ""))}
    except (RuntimeError, ValueError, KeyError, TypeError, AttributeError) as error:
        return {"accepted": False, "detail": str(error)}


def set_automatic(enabled: bool):
    if type(enabled) is not bool or _port is None or _port[4] is None:
        return {"accepted": False, "detail": "Automatic feed control unavailable"}
    return _port[4](enabled)
