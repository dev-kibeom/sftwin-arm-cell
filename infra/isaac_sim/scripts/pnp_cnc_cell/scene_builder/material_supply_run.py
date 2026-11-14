"""Per-delivery spawn execution facts for Isaac Material Supply."""

from __future__ import annotations


class MaterialSupplyRun:
    """Track spawn execution and produce one terminal provider result."""

    def __init__(
        self,
        *,
        delivery_id: str,
        material_type: str,
        requested_quantity: int,
        deadline_ns: int,
    ) -> None:
        if not delivery_id or not material_type:
            raise ValueError("delivery identity and material type are required")
        if type(requested_quantity) is not int or requested_quantity <= 0:
            raise ValueError("requested_quantity must be positive")
        if type(deadline_ns) is not int or deadline_ns <= 0:
            raise ValueError("deadline_ns must be positive")
        self.delivery_id = delivery_id
        self.material_type = material_type
        self.requested_quantity = requested_quantity
        self.deadline_ns = deadline_ns
        self.supplied_paths: set[str] = set()
        self.supply_completed_ns: int | None = None
        self.execution_error: str | None = None
        self.terminal_result: dict[str, object] | None = None
        self.terminal_sim_time_ns: int | None = None
        self.terminal_logged = False

    def record_supplied(self, prim_path: str, *, completed_ns: int) -> None:
        if self.terminal_result is not None:
            return
        if not prim_path or prim_path in self.supplied_paths:
            self.fail_supply("duplicate or invalid spawned prim path")
            return
        self.supplied_paths.add(prim_path)
        if len(self.supplied_paths) > self.requested_quantity:
            self.fail_supply("spawned quantity exceeded request")
        elif len(self.supplied_paths) == self.requested_quantity:
            self.supply_completed_ns = completed_ns

    def fail_supply(self, detail: str) -> None:
        if self.terminal_result is None:
            self.execution_error = detail

    def finish(self, *, now_ns: int, pending_spawns: int) -> dict[str, object] | None:
        if self.terminal_result is not None:
            return None
        expired = now_ns >= self.deadline_ns
        supply_complete = (
            self.execution_error is None
            and pending_spawns == 0
            and len(self.supplied_paths) == self.requested_quantity
            and self.supply_completed_ns is not None
            and self.supply_completed_ns < self.deadline_ns
        )
        complete = supply_complete
        if (
            not complete
            and self.execution_error is None
            and not expired
            and pending_spawns
        ):
            return None
        reason = "NONE" if complete else "SUPPLY"
        return self._terminal_result(
            now_ns=now_ns,
            pending_spawns=pending_spawns,
            completed=complete,
            failure_reason=reason,
        )

    def cancel(self, *, now_ns: int, pending_spawns: int) -> dict[str, object] | None:
        if self.terminal_result is not None:
            return None
        return self._terminal_result(
            now_ns=now_ns,
            pending_spawns=pending_spawns,
            completed=False,
            failure_reason="CANCELED",
        )

    def _terminal_result(
        self,
        *,
        now_ns: int,
        pending_spawns: int,
        completed: bool,
        failure_reason: str,
    ) -> dict[str, object]:
        result: dict[str, object] = {
            "delivery_id": self.delivery_id,
            "material_type": self.material_type,
            "requested_quantity": self.requested_quantity,
            "supplied_quantity": len(self.supplied_paths),
            "completed": completed,
            "valid": True,
            "failure_reason": failure_reason,
            "uncompleted_spawn_count": max(0, pending_spawns),
            "execution_error": self.execution_error,
        }
        self.terminal_sim_time_ns = now_ns
        self.terminal_result = result
        return dict(result)
