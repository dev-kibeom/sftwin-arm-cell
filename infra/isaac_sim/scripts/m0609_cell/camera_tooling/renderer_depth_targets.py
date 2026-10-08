"""Author canonical renderer-depth fixtures in the current unsaved Isaac stage."""

import argparse
import json
import math
import os
from pathlib import Path

from shared.script_editor_bootstrap import is_script_editor_path
from shared.local_artifacts import project_relative_path
from camera_tooling.renderer_depth_target_spec import CANONICAL_FRAME, CANONICAL_TARGETS


ROOT = "/World/SF_Twin_Acceptance/RendererDepth"
TARGETS = CANONICAL_TARGETS
TRANSFORM_TOLERANCE = 1e-6


def q_rotate(q, point):
    x, y, z, w = q
    px, py, pz = point
    ix, iy, iz, iw = (
        w * px + y * pz - z * py,
        w * py + z * px - x * pz,
        w * pz + x * py - y * px,
        -x * px - y * py - z * pz,
    )
    return (
        ix * w - iw * x - iy * z + iz * y,
        iy * w - iw * y - iz * x + ix * z,
        iz * w - iw * z - ix * y + iy * x,
    )


def rotation_matrix_from_quaternion(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def world_point(optical_point, world_to_optical):
    rotated = q_rotate(world_to_optical["quaternion_xyzw"], optical_point)
    return tuple(
        float(world_to_optical["translation_m"][index]) + rotated[index]
        for index in range(3)
    )


def target_placements(snapshot):
    """Derive temporary target centers from canonical optical XYZ + live snapshot."""
    pose = snapshot.get("world_to_optical")
    if (
        not isinstance(pose, dict)
        or pose.get("parent_frame") != "world"
        or pose.get("child_frame") != CANONICAL_FRAME
    ):
        raise RuntimeError(
            "camera snapshot does not identify the canonical optical frame"
        )
    translation = pose.get("translation_m")
    quaternion = pose.get("quaternion_xyzw")
    if (
        not isinstance(translation, (list, tuple))
        or len(translation) != 3
        or not isinstance(quaternion, (list, tuple))
        or len(quaternion) != 4
        or not all(math.isfinite(float(value)) for value in (*translation, *quaternion))
    ):
        raise RuntimeError("camera snapshot has an invalid world-to-optical transform")

    placements = []
    for target in TARGETS:
        front_optical = target.front_optical_xyz_m
        center_optical = (
            front_optical[0],
            front_optical[1],
            front_optical[2] + target.dimensions_m[2] / 2.0,
        )
        placements.append(
            {
                "target": target,
                "front_world_m": world_point(front_optical, pose),
                "center_world_m": world_point(center_optical, pose),
            }
        )
    return tuple(placements)


def matrix_from_optical_pose(pose, center_world):
    from pxr import Gf

    rotation = rotation_matrix_from_quaternion(pose["quaternion_xyzw"])
    matrix = Gf.Matrix4d(1.0)
    # Gf is row-vector based; its upper-left matrix is transpose(world-from-local).
    for row in range(3):
        matrix.SetRow(
            row, Gf.Vec4d(rotation[0][row], rotation[1][row], rotation[2][row], 0.0)
        )
    matrix.SetRow(3, Gf.Vec4d(*center_world, 1.0))
    return matrix


def _matrix_matches(actual, expected, tolerance=TRANSFORM_TOLERANCE):
    return all(
        abs(float(actual[row][column]) - float(expected[row][column])) <= tolerance
        for row in range(4)
        for column in range(4)
    )


def author_targets(stage, snapshot):
    """Replace only the acceptance namespace and verify its three derived targets."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    pose = snapshot["world_to_optical"]
    placements = target_placements(snapshot)
    if stage.GetPrimAtPath(ROOT).IsValid():
        stage.RemovePrim(ROOT)
    UsdGeom.Xform.Define(stage, ROOT)

    expected_matrices = {}
    for placement in placements:
        target = placement["target"]
        path = f"{ROOT}/{target.target_id}"
        xform = UsdGeom.Xform.Define(stage, path)
        expected = matrix_from_optical_pose(pose, placement["center_world_m"])
        xform.AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(expected)
        expected_matrices[target.target_id] = expected

        cube = UsdGeom.Cube.Define(stage, path + "/Surface")
        cube.GetSizeAttr().Set(1.0)
        UsdGeom.Xformable(cube.GetPrim()).AddScaleOp().Set(
            Gf.Vec3f(*target.dimensions_m)
        )
        material = UsdShade.Material.Define(stage, path + "/Material")
        shader = UsdShade.Shader.Define(stage, path + "/Material/Preview")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*target.rgb)
        )
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    root_prim = stage.GetPrimAtPath(ROOT)
    target_ids = sorted(child.GetName() for child in root_prim.GetChildren())
    expected_ids = sorted(target.target_id for target in TARGETS)
    if target_ids != expected_ids:
        raise RuntimeError(
            f"acceptance target namespace mismatch: expected {expected_ids}, got {target_ids}"
        )

    cache = UsdGeom.XformCache()
    target_reports = []
    for placement in placements:
        target = placement["target"]
        path = f"{ROOT}/{target.target_id}"
        prim = stage.GetPrimAtPath(path)
        actual = cache.GetLocalToWorldTransform(prim)
        expected = expected_matrices[target.target_id]
        matches = _matrix_matches(actual, expected)
        if not matches:
            raise RuntimeError(f"authored transform does not match snapshot for {path}")
        target_reports.append(
            {
                "id": target.target_id,
                "path": path,
                "front_optical_xyz_m": list(target.front_optical_xyz_m),
                "front_world_m": list(placement["front_world_m"]),
                "center_world_m": list(placement["center_world_m"]),
                "transform_matches_snapshot": matches,
            }
        )
    return {
        "status": "PASS",
        "namespace": ROOT,
        "frame_id": CANONICAL_FRAME,
        "target_ids": [target.target_id for target in TARGETS],
        "target_paths": [f"{ROOT}/{target.target_id}" for target in TARGETS],
        "targets": target_reports,
        "stage_saved": False,
    }


def create(snapshot_path, stage=None):
    """Author from one current-session snapshot; no planner or manifest is involved."""
    snapshot = json.loads(Path(snapshot_path).read_text())
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no USD stage is open")
    return author_targets(stage, snapshot)


def remove(stage=None):
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if stage is not None and stage.GetPrimAtPath(ROOT).IsValid():
        stage.RemovePrim(ROOT)
    report = {"status": "REMOVED", "namespace": ROOT}
    print(json.dumps(report, indent=2))
    return report


def snapshot_input_path(value, script_path, environ=None):
    if not value:
        raise RuntimeError("camera snapshot path is required")
    path = project_relative_path(value, script_path, environ)
    if not path.is_file():
        raise RuntimeError(f"camera snapshot does not exist: {path}")
    return path


def main(
    argv=None,
    script_path=None,
    environ=None,
    snapshot_path=None,
):
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot")
    parser.add_argument("--remove", action="store_true")
    args, unknown = parser.parse_known_args(argv)
    effective_path = script_path if script_path is not None else __file__
    environment = os.environ if environ is None else environ
    if unknown and not is_script_editor_path(effective_path):
        parser.error("unrecognized arguments: " + " ".join(unknown))
    if args.remove or environment.get("SFTWIN_DEPTH_CALIBRATION_ACTION") == "remove":
        return remove()
    snapshot_value = (
        args.snapshot
        or environment.get("SFTWIN_DEPTH_CALIBRATION_SNAPSHOT")
        or snapshot_path
    )
    snapshot = snapshot_input_path(snapshot_value, effective_path, environment)
    report = create(snapshot)
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    main()
