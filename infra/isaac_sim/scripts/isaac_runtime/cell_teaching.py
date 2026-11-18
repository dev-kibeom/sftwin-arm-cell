"""Optional Cell-owned teaching surface. The Hub never owns robot execution."""

_owner = None


def register(owner):
    global _owner
    _owner = owner


def clear(owner):
    global _owner
    if _owner is owner:
        _owner = None


def status():
    return _owner.status() if _owner is not None else None


def command(operation, **values):
    if _owner is None:
        return {"accepted": False, "detail": "Teaching is unavailable for this Cell."}
    return _owner.command(operation, **values)
