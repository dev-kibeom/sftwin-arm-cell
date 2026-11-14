"""Common physics and bounded observations after a confirmed OPEN detach."""

import logging
import math
import time

LOGGER = logging.getLogger(__name__)


def enable_material_gravity(stage, prim_path):
    """Use the canonical PhysX schema on exactly the detached rigid body."""
    from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics, UsdUtils
    from omni.physx import get_physx_simulation_interface

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f"detached rigid body is unavailable: {prim_path}")
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(False)
    # Removing a fixed joint does not wake a sleeping body when gravity was
    # already enabled. Wake exactly the released body so it resumes falling.
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    get_physx_simulation_interface().wake_up(
        stage_id, PhysicsSchemaTools.sdfPathToInt(prim_path)
    )


class ReleasedMaterialHandler:
    """Activate once per detach; sample only on subsequent physics updates.

    Observation and storage failures never change physical release acceptance.
    One pending record bounds work even if OPEN/CLOSE commands arrive rapidly.
    """

    SAMPLE_LIMIT = 4

    def __init__(self, enable_gravity, observe, submit):
        self._enable_gravity = enable_gravity
        self._observe = observe
        self._submit = submit
        self._pending = None
        self._elapsed_s = 0.0

    def on_detached(self, prim_path, generation):
        self._finish("superseded")
        self._pending = {
            "schema_version": 1,
            "record_type": "isaac_gripper_post_release",
            "record_id": f"post-release-{time.monotonic_ns()}-{generation}",
            "prim_path": str(prim_path),
            "action_graph_command_generation": generation,
            "time_bases": {
                "elapsed_physics_s": "sum of subsequent physics callback dt"
            },
            "samples": [],
            "coverage": {"sample_limit": self.SAMPLE_LIMIT},
        }
        self._elapsed_s = 0.0
        try:
            self._enable_gravity(prim_path)
            result = {"status": "enabled"}
        except Exception as error:
            result = {"status": "failed", "reason": f"{type(error).__name__}: {error}"}
            LOGGER.warning(
                "Post-release gravity activation failed path=%s: %s", prim_path, error
            )
        self._pending["gravity_activation"] = result
        return result

    def on_physics_step(self, dt):
        if self._pending is None or not math.isfinite(dt) or dt <= 0:
            return
        self._elapsed_s += dt
        path = self._pending["prim_path"]
        try:
            body = self._observe(path)
        except Exception as error:
            body = {
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }
        self._pending["samples"].append(
            {
                "elapsed_physics_s": self._elapsed_s,
                "body": body,
            }
        )
        if len(self._pending["samples"]) >= self.SAMPLE_LIMIT:
            self._finish("observed")

    def reset(self):
        self._finish("interrupted")

    def _finish(self, completion):
        record, self._pending = self._pending, None
        if record is None:
            return
        record["completion"] = completion
        record["coverage"]["samples_observed"] = len(record["samples"])
        try:
            self._submit(record)
        except Exception as error:
            LOGGER.warning(
                "Post-release diagnostic storage failed path=%s: %s",
                record["prim_path"],
                error,
            )
