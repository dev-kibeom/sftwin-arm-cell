"""Private in-process scenario ports supplied by the owning source/backend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Literal

Owner = Literal["rgbd_source", "motion_backend"]


@dataclass(frozen=True)
class ScenarioIntent:
    owner: Owner
    scenario: str
    active: bool


class OwnerScenarioPorts:
    """Adapter registry; ports are injected by each owner inside its process.

    This deliberately defines no ROS endpoint or wire format. An absent owner
    port is reported as unavailable and never simulated as a successful action.
    """

    def __init__(
        self,
        *,
        rgbd_source: Callable[[ScenarioIntent], bool] | None = None,
        motion_backend: Callable[[ScenarioIntent], bool] | None = None,
    ) -> None:
        self._ports = {"rgbd_source": rgbd_source, "motion_backend": motion_backend}

    @classmethod
    def from_process_registry(cls) -> "OwnerScenarioPorts":
        """Connect adapters registered by owner code in this same process."""
        return cls(
            rgbd_source=_OWNER_PORTS.get("rgbd_source"),
            motion_backend=_OWNER_PORTS.get("motion_backend"),
        )

    def set_active(self, owner: Owner, scenario: str, active: bool) -> bool:
        allowed = {
            "rgbd_source": {"target_unavailable", "target_stale", "target_invalid"},
            "motion_backend": {"holding_unknown"},
        }
        if scenario not in allowed[owner]:
            raise ValueError(f"scenario is not approved for {owner}: {scenario}")
        port = self._ports[owner]
        if port is None:
            return False
        return bool(port(ScenarioIntent(owner, scenario, active)))


_OWNER_PORTS: dict[Owner, Callable[[ScenarioIntent], bool]] = {}


def bind_owner_scenario_port(
    owner: Owner, port: Callable[[ScenarioIntent], bool]
) -> None:
    """Bind an owner-local in-process adapter during component startup."""
    _OWNER_PORTS[owner] = port
