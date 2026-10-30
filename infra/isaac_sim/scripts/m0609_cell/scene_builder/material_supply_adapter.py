"""Isaac-side consumer that realizes accepted triggers as Physics Drops."""

from __future__ import annotations

import json
import logging
import math
import secrets
import uuid
from dataclasses import asdict
from pathlib import Path
from typing import Any

from scene_builder.material_supply import MaterialSupplyProfile, sample_initial_poses
from scene_builder.material_supply_pose_recorder import RawPartPoseRecorder
from scene_builder.cuboid_demonstration import bind_adapter
from scene_builder.material_supply_run import MaterialSupplyRun
from scene_builder.material_supply_sidecar import simulation_time_ns


CELL_ROOT = "/World/SF_Twin_Cell"
DELIVERY_ROOT = f"{CELL_ROOT}/MaterialSupply/Deliveries"


def _world_pose_to_local_transform(
    gf: Any,
    position: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    parent_inverse: Any,
) -> Any:
    world_transform = gf.Matrix4d(1.0)
    world_transform.SetRotate(
        gf.Quatd(
            orientation_xyzw[3],
            gf.Vec3d(*orientation_xyzw[:3]),
        )
    )
    world_transform.SetTranslateOnly(gf.Vec3d(*position))
    return world_transform * parent_inverse


def _delivery_prim_path(delivery_id: str) -> str:
    """Return a USD-safe prim path while keeping the delivery UUID intact."""
    return f"{DELIVERY_ROOT}/Delivery_{delivery_id}"


class IsaacMaterialSupplyAdapter:
    """Consume Integration's accepted event and create gravity-enabled RawParts."""

    def __init__(
        self,
        *,
        stage: Any,
        sidecar: Any,
        profile: MaterialSupplyProfile,
        shelf_origin_m: tuple[float, float, float],
        delivery_deadline_ms: int,
        artifact_directory: Path = Path(".local_artifacts/isaac_pose_recordings"),
        timeline: Any | None = None,
    ) -> None:
        self.stage = stage
        self.sidecar = sidecar
        self.timeline = timeline
        self.profile = profile
        self.pose_recorder = RawPartPoseRecorder(stage, artifact_directory)
        self.shelf_origin_m = shelf_origin_m
        if type(delivery_deadline_ms) is not int or delivery_deadline_ms <= 0:
            raise ValueError("delivery_deadline_ms must be a positive integer")
        self.delivery_deadline_ns = delivery_deadline_ms * 1_000_000
        self._deliveries: set[str] = set()
        self._delivery_paths: set[str] = set()
        self._demonstration_paths: set[str] = set()
        self._demonstration_loaded = False
        self._runs: dict[str, MaterialSupplyRun] = {}
        self._pending_parts: list[tuple[str, int, Any, int]] = []
        self._closed = False
        self._sidecar_unavailable_reported = False
        self._stop_reset_pending = False
        self._terminal_pending_counts: dict[str, int] = {}
        self._logger = logging.getLogger(__name__)
        self._pose_recorder_error_reported = False
        bind_adapter(self)

    def spin_once(self) -> None:
        """Pump sidecar messages and author USD on Kit's update thread."""
        if self._closed or self._stop_reset_pending:
            return
        self._record_rawpart_poses()
        if not self.sidecar.available:
            self._abort_incomplete_deliveries()
            return
        for message in self.sidecar.poll_messages():
            if message.get("op") == "accepted_handoff":
                self._on_accepted_trigger(message)
            else:
                self._logger.warning(
                    "Material sidecar message rejected: unsupported operation"
                )
        if not self.sidecar.available:
            self._abort_incomplete_deliveries()
            return
        self.sidecar.flush_outgoing()
        if not self.sidecar.available:
            self._abort_incomplete_deliveries()
            return
        self._spawn_one_if_due()
        self._finish_deliveries()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.pose_recorder.stop()
        from scene_builder import cuboid_demonstration

        if cuboid_demonstration._adapter is self:
            bind_adapter(None)
        # Missing completion evidence keeps Integration fail-closed on shutdown.
        self._pending_parts.clear()
        self._runs.clear()
        self.sidecar.close()

    def set_live_test_session_id(self, session_id: str | None) -> None:
        """Bind pose observations to the active Isaac Play-to-Stop session."""
        self.pose_recorder.bind_live_test_session(session_id)

    def reset_generated_materials(self, attachment_reset=None) -> bool:
        """Reset attachment ownership before material and report canceled work."""
        self._stop_reset_pending = True
        self.pose_recorder.stop()
        for run in self._runs.values():
            delivery_id = run.delivery_id
            self._terminal_pending_counts.setdefault(
                delivery_id,
                sum(item[0] == delivery_id for item in self._pending_parts),
            )
        self._pending_parts.clear()
        if callable(attachment_reset):
            try:
                if attachment_reset() is False:
                    self._logger.error(
                        "material reset deferred: Gripper attachment reset failed"
                    )
                    return False
            except Exception as error:
                self._logger.error(
                    f"material reset deferred: Gripper attachment reset failed: {error}"
                )
                return False

        failed_paths: set[str] = set()
        for path in tuple(self._delivery_paths):
            try:
                self.stage.RemovePrim(path)
            except Exception as error:
                failed_paths.add(path)
                self._logger.error(
                    f"failed to remove generated material delivery path={path}: {error}"
                )
        for path in tuple(self._delivery_paths.difference(failed_paths)):
            if self._prim_exists(path):
                failed_paths.add(path)
                self._logger.error(
                    f"generated material delivery path remains after Stop: {path}"
                )
        self._delivery_paths = failed_paths
        failed_demo_paths: set[str] = set()
        for path in tuple(self._demonstration_paths):
            try:
                self.stage.RemovePrim(path)
            except Exception as error:
                failed_demo_paths.add(path)
                self._logger.error(
                    "failed to remove Demonstration materials: %s", error
                )
        for path in tuple(self._demonstration_paths.difference(failed_demo_paths)):
            if self._prim_exists(path):
                failed_demo_paths.add(path)
        self._demonstration_paths = failed_demo_paths
        if not failed_demo_paths:
            self._demonstration_loaded = False
        remaining_parts = {
            part_path
            for run in self._runs.values()
            for part_path in run.supplied_paths
            if self._prim_exists(part_path)
        }
        if failed_paths or failed_demo_paths or remaining_parts:
            self._logger.error(
                "material reset incomplete; keeping unfinished deliveries for retry "
                f"failed_delivery_roots={len(failed_paths)} "
                f"remaining_parts={len(remaining_parts)} "
                f"failed_demonstration_roots={len(failed_demo_paths)}"
            )
            return False

        for run in tuple(self._runs.values()):
            delivery_id = run.delivery_id
            result = run.cancel(
                now_ns=self._now_ns(),
                pending_spawns=self._terminal_pending_counts.get(delivery_id, 0),
            )
            if result is None:
                result = run.terminal_result
            if result is not None and not self._terminalize_delivery(
                delivery_id, result
            ):
                return False
        self._runs.clear()
        self._stop_reset_pending = False
        return True

    def load_demonstration(self, scenario_id: str) -> int:
        """Create one preset during a Play session, without a supply trigger."""
        unavailable = self.demonstration_load_unavailable_reason()
        if unavailable is not None:
            raise RuntimeError(unavailable)
        from scene_builder.cuboid_demonstrations import build_layout

        parts = build_layout(
            scenario_id, self.profile.dimensions_m, self.shelf_origin_m
        )
        self._demonstration_loaded = True
        try:
            return self._create_demonstration_parts(parts)
        except Exception:
            if not self._demonstration_paths:
                self._demonstration_loaded = False
            raise

    def demonstration_load_unavailable_reason(self) -> str | None:
        if self.timeline is None or not self.timeline.is_playing():
            return "Load Demonstration requires Timeline Play"
        if self._demonstration_loaded:
            return "Demonstration already loaded in this Play session"
        if self._stop_reset_pending:
            return "Material Supply reset is still in progress"
        if (
            self._delivery_paths
            or self._demonstration_paths
            or self._runs
            or self._pending_parts
        ):
            return "Demonstration requires an empty Material Supply"
        if any(
            prim.IsValid()
            and prim.GetName().startswith("RawPart_")
            and "/MaterialSupply/Deliveries/" in str(prim.GetPath())
            for prim in self.stage.Traverse()
        ):
            return "Demonstration requires all existing RawParts to be removed"
        return None

    def _create_demonstration_parts(self, parts: list[dict[str, Any]]) -> int:
        if not parts:
            raise ValueError("Demonstration contains no RawParts")
        if self._stop_reset_pending:
            raise RuntimeError("Demonstration requires Timeline Play")
        demo_id = uuid.uuid4().hex
        demo_root = f"{DELIVERY_ROOT}/Demonstration_{demo_id}"
        try:
            from pxr import UsdGeom

            UsdGeom.Xform.Define(self.stage, demo_root)
            self._demonstration_paths.add(demo_root)
            parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(
                self.stage.GetPrimAtPath(demo_root)
            )
            parent_inverse = parent_world.GetInverse()
            from pxr import Gf

            for index, part in enumerate(parts):
                position = tuple(float(value) for value in part["position_xyz"])
                orientation = tuple(float(value) for value in part["orientation_xyzw"])
                dimensions = tuple(
                    float(value)
                    for value in part.get("dimensions_m", self.profile.dimensions_m)
                )
                mass = float(part.get("mass_kg", self.profile.mass_kg))
                if (
                    len(position) != 3
                    or len(orientation) != 4
                    or len(dimensions) != 3
                    or not all(
                        math.isfinite(value)
                        for value in (*position, *orientation, *dimensions, mass)
                    )
                    or any(value <= 0 for value in dimensions)
                    or mass <= 0
                ):
                    raise ValueError(
                        "Demonstration poses must contain finite XYZ and XYZW"
                    )
                local_transform = _world_pose_to_local_transform(
                    Gf, position, orientation, parent_inverse
                )
                self._create_part(
                    demo_id,
                    index,
                    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    part_root_path=demo_root,
                    demonstration_position=position,
                    demonstration_orientation=orientation,
                    demonstration_transform=local_transform,
                    dimensions_m=dimensions,
                    mass_kg=mass,
                )
        except Exception:
            try:
                self.stage.RemovePrim(demo_root)
                if self._prim_exists(demo_root):
                    self._logger.error(
                        "partial Demonstration remains after removal: %s", demo_root
                    )
                else:
                    self._demonstration_paths.discard(demo_root)
                    self._demonstration_loaded = False
            except Exception as cleanup_error:
                self._logger.error(
                    "failed to remove partial Demonstration: %s", cleanup_error
                )
            raise
        return len(parts)

    def _record_rawpart_poses(self) -> None:
        try:
            if self.timeline is None or not self.timeline.is_playing():
                return
            import omni.physx

            self.pose_recorder.sample(
                omni.physx.get_physx_interface(),
                float(self.timeline.get_current_time()),
                now=__import__("time").monotonic(),
            )
            self._pose_recorder_error_reported = False
        except (
            ImportError,
            AttributeError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
        ):
            if not self._pose_recorder_error_reported:
                self._logger.warning("Isaac RawPart pose diagnostics are unavailable")
                self._pose_recorder_error_reported = True

    def _prim_exists(self, path: str) -> bool:
        try:
            return bool(self.stage.GetPrimAtPath(path).IsValid())
        except Exception as error:
            self._logger.error(
                f"cannot observe post-reset material path={path}: {error}"
            )
            return True

    def _abort_incomplete_deliveries(self) -> None:
        if not self._sidecar_unavailable_reported:
            self._logger.error(
                "material supply ROS sidecar unavailable; incomplete deliveries "
                "will remain without successful completion evidence"
            )
            self._sidecar_unavailable_reported = True
        self._pending_parts.clear()
        self._runs.clear()

    def _on_accepted_trigger(self, trigger: Any) -> None:
        required = {
            "delivery_id",
            "material_type",
            "quantity",
            "valid",
            "has_random_seed",
        }
        if not required.issubset(trigger):
            self._logger.warning(
                "Accepted material handoff rejected: incomplete IPC message"
            )
            return
        if (
            type(trigger["valid"]) is not bool
            or not isinstance(trigger["material_type"], str)
            or type(trigger["has_random_seed"]) is not bool
        ):
            self._logger.warning(
                "Accepted material handoff rejected: malformed IPC message"
            )
            return
        try:
            delivery_id = _delivery_key(trigger["delivery_id"])
        except (TypeError, ValueError) as error:
            self._logger.warning(
                f"ignoring accepted-handoff event with invalid identity: {error}"
            )
            return
        if not trigger["valid"]:
            self._logger.warning("ignoring invalid accepted-handoff event")
            return
        now_ns = self._now_ns()
        if trigger["material_type"] != self.profile.material_type:
            self._logger.error(
                f"accepted material type is not configured: {trigger['material_type']}"
            )
            return
        if (
            type(trigger["quantity"]) is not int
            or trigger["quantity"] <= 0
            or trigger["quantity"] > 0xFFFFFFFF
        ):
            self._logger.error("accepted quantity must be positive")
            return
        if delivery_id in self._deliveries or self._delivery_prim_exists(delivery_id):
            self._deliveries.add(delivery_id)
            self._logger.debug("duplicate accepted-handoff event ignored")
            return

        seed_was_supplied = bool(trigger["has_random_seed"])
        if seed_was_supplied and (
            type(trigger.get("random_seed")) is not int
            or not 0 <= trigger["random_seed"] <= 0xFFFFFFFFFFFFFFFF
        ):
            self._logger.error("ignoring invalid random seed")
            return
        seed = (
            int(trigger["random_seed"]) if seed_was_supplied else secrets.randbits(64)
        )
        poses = sample_initial_poses(
            self.profile,
            int(trigger["quantity"]),
            shelf_origin_m=self.shelf_origin_m,
            seed=seed,
        )
        self._deliveries.add(delivery_id)
        self._delivery_paths.add(_delivery_prim_path(delivery_id))
        self._runs[delivery_id] = MaterialSupplyRun(
            delivery_id=delivery_id,
            material_type=self.profile.material_type,
            requested_quantity=int(trigger["quantity"]),
            deadline_ns=now_ns + self.delivery_deadline_ns,
        )
        try:
            self._author_delivery_metadata(
                delivery_id,
                int(trigger["quantity"]),
                seed,
                seed_was_supplied,
            )
            now_ns = self._now_ns()
            for index, pose in enumerate(poses):
                self._pending_parts.append((delivery_id, index, pose, now_ns))
        except Exception as error:
            self._runs[delivery_id].fail_supply(f"metadata_error={error}")
            return
        self._logger.info(
            "material Physics Drop scheduled "
            f"material_type={self.profile.material_type} "
            f"quantity={int(trigger['quantity'])}"
        )

    def _spawn_one_if_due(self) -> None:
        if not self._pending_parts:
            return
        delivery_id, index, pose, due_ns = self._pending_parts[0]
        run = self._runs.get(delivery_id)
        if run is None or run.terminal_result is not None:
            self._pending_parts = [
                item for item in self._pending_parts if item[0] != delivery_id
            ]
            return
        now_ns = self._now_ns()
        if now_ns >= run.deadline_ns:
            return
        if now_ns < due_ns:
            return
        try:
            part_path = self._create_part(delivery_id, index, pose)
        except Exception as error:
            run.fail_supply(f"sequence={index} error={error}")
            return
        run.record_supplied(part_path, completed_ns=self._now_ns())
        self._pending_parts.pop(0)
        if self._pending_parts:
            next_delivery, next_index, next_pose, _ = self._pending_parts[0]
            self._pending_parts[0] = (
                next_delivery,
                next_index,
                next_pose,
                now_ns + int(self.profile.spawn_interval_s * 1_000_000_000),
            )

    def _finish_deliveries(self) -> None:
        """Finish deliveries from spawn execution facts and simulation deadline."""
        now_ns = self._now_ns()
        for delivery_id, run in tuple(self._runs.items()):
            pending_count = sum(item[0] == delivery_id for item in self._pending_parts)
            result = run.finish(now_ns=now_ns, pending_spawns=pending_count)
            if result is not None:
                self._terminalize_delivery(delivery_id, result)

    def _terminalize_delivery(
        self, delivery_id: str, result: dict[str, object]
    ) -> bool:
        """Close one delivery's spawn lifecycle exactly once."""
        run = self._runs.get(delivery_id)
        if run is None:
            return True
        if run.terminal_result is None:
            run.terminal_result = dict(result)
        terminal_result = run.terminal_result
        pending_count = self._terminal_pending_counts.pop(
            delivery_id,
            sum(item[0] == delivery_id for item in self._pending_parts),
        )
        self._pending_parts = [
            item for item in self._pending_parts if item[0] != delivery_id
        ]
        terminal_time_ns = run.terminal_sim_time_ns
        if not run.terminal_logged:
            completed = terminal_result.get("completed") is True
            event = (
                "Material supply completed"
                if completed
                else "Material supply terminated without full completion"
            )
            fields = (
                f"delivery_id={delivery_id} "
                f"requested_quantity={terminal_result.get('requested_quantity')} "
                f"supplied_quantity={terminal_result.get('supplied_quantity')} "
                f"failure_reason={terminal_result.get('failure_reason')} "
                f"uncompleted_spawn_count={pending_count}"
            )
            if (
                not completed
                and terminal_time_ns is not None
                and terminal_time_ns >= run.deadline_ns
            ):
                fields += (
                    f" simulation_time_ns={terminal_time_ns}"
                    f" deadline_ns={run.deadline_ns}"
                )
            if terminal_result.get("execution_error"):
                detail = " ".join(str(terminal_result["execution_error"]).split())[:240]
                fields += f" execution_error={detail}"
            if completed:
                self._logger.info("%s: %s", event, fields)
            else:
                self._logger.warning("%s: %s", event, fields)
            run.terminal_logged = True

        if not self._publish_completion(terminal_result):
            return False
        self._runs.pop(delivery_id, None)
        return True

    def _publish_completion(self, result: dict[str, object]) -> bool:
        if not self.sidecar.available:
            return False
        try:
            self.sidecar.send_completion(dict(result))
        except Exception as error:
            self._logger.error(f"failed to publish transfer completion: {error}")
            return False
        return self.sidecar.available

    @staticmethod
    def _now_ns() -> int:
        return simulation_time_ns()

    def _delivery_prim_exists(self, delivery_id: str) -> bool:
        try:
            prim = self.stage.GetPrimAtPath(_delivery_prim_path(delivery_id))
            return bool(prim.IsValid())
        except Exception as error:
            self._logger.error(
                "cannot establish duplicate-delivery state from the current stage: "
                f"{error}"
            )
            return True

    def _author_delivery_metadata(
        self,
        delivery_id: str,
        quantity: int,
        seed: int,
        seed_was_supplied: bool,
    ) -> None:
        from pxr import Sdf, UsdGeom

        path = _delivery_prim_path(delivery_id)
        prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
        metadata = {
            "delivery_id": (Sdf.ValueTypeNames.String, delivery_id),
            "requested_quantity": (Sdf.ValueTypeNames.UInt, quantity),
            "random_seed_used": (Sdf.ValueTypeNames.UInt64, seed),
            "random_seed_was_supplied": (Sdf.ValueTypeNames.Bool, seed_was_supplied),
            "effective_supply_profile_json": (
                Sdf.ValueTypeNames.String,
                json.dumps(asdict(self.profile), sort_keys=True),
            ),
        }
        for name, (value_type, value) in metadata.items():
            prim.CreateAttribute(f"sf_twin:{name}", value_type, custom=True).Set(value)

    def _create_part(
        self,
        delivery_id: str,
        index: int,
        pose: tuple[float, float, float, float, float, float],
        *,
        part_root_path: str | None = None,
        demonstration_position: tuple[float, float, float] | None = None,
        demonstration_orientation: tuple[float, float, float, float] | None = None,
        demonstration_transform: Any | None = None,
        dimensions_m: tuple[float, float, float] | None = None,
        mass_kg: float | None = None,
    ) -> str:
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        dimensions = dimensions_m or self.profile.dimensions_m
        mass = self.profile.mass_kg if mass_kg is None else mass_kg
        root_path = part_root_path or _delivery_prim_path(delivery_id)
        part_path = f"{root_path}/RawPart_{index:04d}"
        part = UsdGeom.Xform.Define(self.stage, part_path)
        prim = part.GetPrim()
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        if demonstration_transform is not None:
            xform.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                demonstration_transform
            )
        elif demonstration_position is None:
            xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(*pose[:3])
            )
            rotation = (
                Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.RadiansToDegrees(pose[5]))
                * Gf.Rotation(Gf.Vec3d(0, 1, 0), Gf.RadiansToDegrees(pose[4]))
                * Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.RadiansToDegrees(pose[3]))
            )
            xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(rotation.GetQuat())
            )
        else:
            assert demonstration_orientation is not None
            xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(*demonstration_position)
            )
            xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(
                    demonstration_orientation[3],
                    Gf.Vec3d(*demonstration_orientation[:3]),
                )
            )

        shape = UsdGeom.Cube.Define(self.stage, f"{part_path}/collision")
        shape.CreateSizeAttr(1.0)
        shape_xform = UsdGeom.Xformable(shape.GetPrim())
        shape_xform.ClearXformOpOrder()
        shape_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*dimensions)
        )

        color = Gf.Vec3f(0.42, 0.30, 0.18)
        UsdGeom.Gprim(shape.GetPrim()).CreateDisplayColorPrimvar().Set([color])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_body.CreateDisableGravityAttr(False)
        if demonstration_position is None:
            prim.CreateAttribute(
                "sf_twin:delivery_id", Sdf.ValueTypeNames.String, custom=True
            ).Set(delivery_id)
        prim.CreateAttribute(
            "sf_twin:material_type", Sdf.ValueTypeNames.String, custom=True
        ).Set(self.profile.material_type)
        prim.CreateAttribute(
            "sf_twin:shape", Sdf.ValueTypeNames.String, custom=True
        ).Set(self.profile.shape)
        if demonstration_position is None:
            prim.CreateAttribute(
                "sf_twin:delivery_sequence", Sdf.ValueTypeNames.UInt, custom=True
            ).Set(index)
        prim.CreateAttribute(
            "sf_twin:dimensions_m", Sdf.ValueTypeNames.Double3, custom=True
        ).Set(Gf.Vec3d(*dimensions))
        if demonstration_position is not None:
            prim.CreateAttribute(
                "sf_twin:cuboid_demonstration_id",
                Sdf.ValueTypeNames.String,
                custom=True,
            ).Set(delivery_id)
        else:
            prim.CreateAttribute(
                "sf_twin:sampled_initial_position",
                Sdf.ValueTypeNames.Double3,
                custom=True,
            ).Set(Gf.Vec3d(*pose[:3]))
            prim.CreateAttribute(
                "sf_twin:sampled_initial_orientation_rpy",
                Sdf.ValueTypeNames.Double3,
                custom=True,
            ).Set(Gf.Vec3d(*pose[3:]))
        return part_path


def _delivery_key(raw_uuid: Any) -> str:
    raw = (
        bytes.fromhex(raw_uuid)
        if isinstance(raw_uuid, str)
        else bytes(int(value) for value in raw_uuid)
    )
    if len(raw) != 16:
        raise ValueError("delivery_id must contain 16 UUID bytes")
    if not any(raw):
        raise ValueError("delivery_id must be nonzero")
    return raw.hex()
