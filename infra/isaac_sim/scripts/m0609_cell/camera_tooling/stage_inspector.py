"""Read-only Isaac stage inspection and camera snapshot export for M0609."""

import argparse
import json
import math
import os
from pathlib import Path

from camera_tooling.inspection_math import *  # compatibility re-export for inspect_camera
from shared.script_editor_bootstrap import is_script_editor_path
from shared.local_artifacts import output_path


def collision_scene_query_snapshot(camera_transform, reference_geometry):
    """Raycast reference rays through PhysX collision geometry, never renderer truth.

    PhysX scene query is synchronous and useful to distinguish an authored point
    from a collision surface.  It is deliberately labelled as a limitation:
    collision meshes, visibility masks, materials, and renderer tessellation can
    differ from the Replicator render product that produced ROS depth.
    """
    try:
        import omni.physx
        from pxr import Gf

        interface = omni.physx.get_physx_scene_query_interface()
        origin = tuple(
            float(value)
            for value in camera_transform.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        )
        inverse_camera = camera_transform.GetInverse()
        entries = []
        for geometry in reference_geometry:
            for point_name, world_point in geometry["world_surface_points_m"].items():
                direction, maximum_distance = ray_direction(origin, world_point)
                hit = interface.raycast_closest(
                    origin, direction, maximum_distance + 10.0
                )
                entry = {
                    "reference_prim_path": geometry["prim_path"],
                    "reference_point": point_name,
                    "reference_world_point_m": world_point,
                    "reference_optical_z_m": usd_camera_to_optical(
                        tuple(
                            float(value)
                            for value in inverse_camera.Transform(
                                Gf.Vec3d(*world_point)
                            )
                        )
                    )[2],
                    "ray_origin_world_m": list(origin),
                    "ray_direction_world": list(direction),
                    "ray_kind": "authored-reference direction; not an exact discrete ROS pixel-center ray",
                    "query_kind": "PhysX collision scene query; not renderer-visible ground truth",
                }
                if not hit.get("hit", False):
                    entries.append(
                        {
                            **entry,
                            "status": "NOT VERIFIED",
                            "reason": "collision scene query returned no hit",
                        }
                    )
                    continue
                hit_point = [float(value) for value in hit["position"]]
                hit_usd = inverse_camera.Transform(Gf.Vec3d(*hit_point))
                entries.append(
                    {
                        **entry,
                        "status": "PARTIALLY VERIFIED",
                        "hit_prim_path": str(hit.get("rigidBody", "")),
                        "hit_world_point_m": hit_point,
                        "hit_optical_z_m": usd_camera_to_optical(
                            tuple(float(value) for value in hit_usd)
                        )[2],
                        "hit_distance_m": float(
                            hit.get(
                                "distance",
                                hit.get("hit_t", math.dist(origin, hit_point)),
                            )
                        ),
                        "limitation": "Collision hit is not accepted as Replicator renderer-visible surface ground truth.",
                    }
                )
        return {
            "status": "PARTIALLY VERIFIED",
            "query_kind": "PhysX collision scene query",
            "limitation": "Collision geometry can differ from renderer-visible geometry; do not use these hits to verify depth semantics.",
            "entries": entries,
        }
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
        print(f"[WARN] Optional collision scene-query diagnostic failed: {reason}")
        return {
            "status": "NOT VERIFIED",
            "reason": reason,
            "limitation": "No renderer-visible surface query was produced.",
        }


def ros_camera_rotations(world_from_usd_camera):
    """Derive ROS optical/link axes from the USD camera-local axes.

    USD camera coordinates are right/up/negative-forward. ROS optical is
    right/down/positive-forward. The composed USD transform is therefore first
    converted with the explicit USD-to-optical point basis; camera_link is
    derived afterwards.
    """
    world_from_optical = multiply(
        world_from_usd_camera, USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS
    )
    # T_link_optical stores transpose(CAMERA_LINK_TO_OPTICAL_BASIS). Therefore
    # T_world_link = T_world_optical * inverse(T_link_optical) uses the point
    # coordinate matrix itself here.
    world_from_link = multiply(world_from_optical, CAMERA_LINK_TO_OPTICAL_BASIS)
    return world_from_link, world_from_optical


def render_product_camera_bindings(stage, camera_path):
    """Read Render Product camera relationships without changing the stage."""
    from pxr import UsdRender

    bindings = []
    for prim in stage.Traverse():
        product = UsdRender.Product(prim)
        if not product:
            continue
        relationship = product.GetCameraRel()
        targets = (
            [str(target) for target in relationship.GetTargets()]
            if relationship
            else []
        )
        bindings.append(
            {
                "render_product": str(prim.GetPath()),
                "camera_targets": targets,
                "uses_camera_sensor": camera_path in targets,
            }
        )
    return bindings


def composed_camera_transform_diagnostics(prim, xform_cache):
    """Return raw composed affine transform evidence without pose extraction."""
    transform = xform_cache.GetLocalToWorldTransform(prim)
    raw_matrix = [
        [float(transform[row][column]) for column in range(4)] for row in range(4)
    ]
    # The raw matrix is row-vector based.  Its transpose is the conventional
    # world-from-local rotation representation used only after rigidity passes.
    raw_upper_left = tuple(
        tuple(raw_matrix[row][column] for column in range(3)) for row in range(3)
    )
    world_from_local = transpose(raw_upper_left)
    return {
        "prim_path": str(prim.GetPath()),
        "raw_matrix_row_major": raw_matrix,
        "translation_m": [float(transform[3][index]) for index in range(3)],
        "affine_3x3": affine_3x3_diagnostics(raw_upper_left),
    }, world_from_local


def composed_camera_pose(prim, xform_cache):
    """Return a pose only after the raw composed transform proves rigid."""
    diagnostic, rotation = composed_camera_transform_diagnostics(prim, xform_cache)
    if not diagnostic["affine_3x3"]["is_rigid_rotation"]:
        raise RuntimeError(
            f"camera prim has a non-rigid composed transform: {prim.GetPath()}"
        )
    diagnostic["quaternion_xyzw"] = list(quaternion_xyzw_from_rotation(rotation))
    diagnostic["rotation_matrix_world_from_local"] = [list(row) for row in rotation]
    return diagnostic, rotation


def parent_local_transform_chain(prim):
    """Return each ancestor's local transform; diagnostics never mutate stage state."""
    from pxr import UsdGeom

    chain = []
    current = prim
    while current and current.IsValid():
        record = {
            "prim_path": str(current.GetPath()),
            "type_name": current.GetTypeName(),
        }
        try:
            local = UsdGeom.Xformable(current).GetLocalTransformation()
            if isinstance(local, tuple):
                local = local[0]
            record["local_matrix_row_major"] = [
                [float(local[row][column]) for column in range(4)] for row in range(4)
            ]
        except Exception as error:
            record["local_transform_status"] = "NOT VERIFIED"
            record["reason"] = f"{type(error).__name__}: {error}"
        chain.append(record)
        current = current.GetParent()
    return chain


def d455_color_camera_pose_comparison(stage, sensor_prim, xform_cache):
    """Read and compare the D455 Color camera composed pose with Camera_Sensor.

    The D455 asset remains an externally authored referenced subtree.  This
    inspection only reports its composed pose and does not make it a ROS graph
    source or mutate its transforms.
    """
    from pxr import UsdGeom

    sensor_diagnostic, sensor_rotation = composed_camera_transform_diagnostics(
        sensor_prim, xform_cache
    )
    sensor_diagnostic["parent_local_transform_chain"] = parent_local_transform_chain(
        sensor_prim
    )
    root = stage.GetPrimAtPath(D455_ROOT_PATH)
    if not root.IsValid():
        return {
            "status": "NOT VERIFIED",
            "reason": f"D455 root not found: {D455_ROOT_PATH}",
            "camera_sensor": sensor_diagnostic,
        }

    cameras = []
    selected = None
    selected_rotation = None
    for prim in stage.Traverse():
        # Stage traversal is retained for API compatibility across Isaac builds;
        # only descendants of the referenced D455 root are considered.
        path = str(prim.GetPath())
        if not path.startswith(D455_ROOT_PATH + "/") or not prim.IsA(UsdGeom.Camera):
            continue
        pose, rotation = composed_camera_transform_diagnostics(prim, xform_cache)
        pose["parent_local_transform_chain"] = parent_local_transform_chain(prim)
        cameras.append(pose)
        if prim.GetName() == D455_COLOR_CAMERA_NAME:
            selected = pose
            selected_rotation = rotation

    if selected is None:
        return {
            "status": "NOT VERIFIED",
            "reason": f"D455 Color camera not found by name: {D455_COLOR_CAMERA_NAME}",
            "d455_root": D455_ROOT_PATH,
            "camera_sensor": sensor_diagnostic,
            "discovered_d455_cameras": cameras,
        }

    raw_comparison = matrix_difference(
        sensor_diagnostic["raw_matrix_row_major"], selected["raw_matrix_row_major"]
    )
    if (
        not sensor_diagnostic["affine_3x3"]["is_rigid_rotation"]
        or not selected["affine_3x3"]["is_rigid_rotation"]
    ):
        return {
            "status": "NOT VERIFIED",
            "reason": "pose/quaternion comparison requires rigid composed transforms; raw matrix comparison exported without orthonormalization",
            "d455_root": D455_ROOT_PATH,
            "camera_sensor": sensor_diagnostic,
            "d455_color_camera": selected,
            "raw_composed_matrix_comparison": raw_comparison,
            "pose_comparison": {
                "status": "NOT VERIFIED",
                "reason": "at least one composed transform is non-rigid",
            },
            "discovered_d455_cameras": cameras,
        }

    sensor_diagnostic["quaternion_xyzw"] = list(
        quaternion_xyzw_from_rotation(sensor_rotation)
    )
    sensor_diagnostic["rotation_matrix_world_from_local"] = [
        list(row) for row in sensor_rotation
    ]
    selected["quaternion_xyzw"] = list(quaternion_xyzw_from_rotation(selected_rotation))
    selected["rotation_matrix_world_from_local"] = [
        list(row) for row in selected_rotation
    ]
    return {
        "status": "VERIFIED",
        "d455_root": D455_ROOT_PATH,
        "camera_sensor": sensor_diagnostic,
        "d455_color_camera": selected,
        "raw_composed_matrix_comparison": raw_comparison,
        "pose_comparison": {
            "status": "VERIFIED",
            "camera_sensor_relative_to_d455_color": pose_difference(
                selected_rotation,
                selected["translation_m"],
                sensor_rotation,
                sensor_diagnostic["translation_m"],
            ),
        },
        "discovered_d455_cameras": cameras,
        "interpretation": "Raw composed USD local-frame pose comparison only; render projection and effective sensor axes are reported separately.",
    }


def graph_connection_descriptor(connection):
    """Describe an OmniGraph Attribute through its owning Node, never as a Node."""
    descriptor = {
        "python_type": f"{type(connection).__module__}.{type(connection).__name__}"
    }
    descriptor["available_api"] = [
        name
        for name in ("get_node", "get_name", "get_prim_path")
        if callable(getattr(connection, name, None))
    ]
    get_node = getattr(connection, "get_node", None)
    if not callable(get_node):
        raise RuntimeError(
            "upstream connection is not an Attribute with get_node(): "
            + descriptor["python_type"]
            + "; available API: "
            + repr(descriptor["available_api"])
        )
    node = get_node()
    descriptor["attribute_name"] = (
        str(connection.get_name())
        if callable(getattr(connection, "get_name", None))
        else None
    )
    descriptor["owning_node_type"] = f"{type(node).__module__}.{type(node).__name__}"
    node_path = getattr(node, "get_prim_path", None)
    if not callable(node_path):
        node_api = [
            name
            for name in ("get_prim_path", "get_name")
            if callable(getattr(node, name, None))
        ]
        raise RuntimeError(
            "owning OmniGraph Node has no get_prim_path(): "
            + descriptor["owning_node_type"]
            + "; available API: "
            + repr(node_api)
        )
    descriptor["owning_node_path"] = str(node_path())
    return descriptor


def depth_helper_runtime_snapshot(stage):
    """Read the existing helper inputs and its Render Product connection.

    ROS2CameraHelper creates its Replicator writer dynamically, so its renderVar
    is not an authored Action Graph edge. The static source mapping is reported
    separately from the live graph connection instead of being presented as an
    observed runtime edge.
    """
    try:
        import omni.graph.core as og

        node = og.Controller.node(DEPTH_HELPER_PATH)
        if not node or not node.is_valid():
            return {
                "status": "NOT VERIFIED",
                "reason": f"depth helper not found: {DEPTH_HELPER_PATH}",
            }
        render_product_input = node.get_attribute("inputs:renderProductPath")
        upstream = render_product_input.get_upstream_connections()
        render_product_path = render_product_input.get()
        render_product_path = str(render_product_path) if render_product_path else None
        product = (
            stage.GetPrimAtPath(render_product_path) if render_product_path else None
        )
        targets = []
        if product and product.IsValid():
            from pxr import UsdRender

            relationship = UsdRender.Product(product).GetCameraRel()
            targets = (
                [str(target) for target in relationship.GetTargets()]
                if relationship
                else []
            )
        return {
            "status": (
                "VERIFIED"
                if render_product_path and CAMERA_PATH in targets
                else "NOT VERIFIED"
            ),
            "helper_path": DEPTH_HELPER_PATH,
            "helper_type_token": str(node.get_attribute("inputs:type").get()),
            "topic_name": str(node.get_attribute("inputs:topicName").get()),
            "frame_id": str(node.get_attribute("inputs:frameId").get()),
            "render_product_path": render_product_path,
            "render_product_input_upstream": [
                graph_connection_descriptor(attribute) for attribute in upstream
            ],
            "render_product_camera_targets": targets,
            "uses_camera_sensor": CAMERA_PATH in targets,
            "render_var_runtime_edge": "NOT VERIFIED: ROS2CameraHelper creates the writer dynamically",
            "source_mapping": "type=depth maps to SensorType.DistanceToImagePlane in ROS2CameraHelper source",
        }
    except Exception as error:
        reason = f"{type(error).__name__}: {error}"
        print(f"[WARN] Optional depth helper runtime diagnostic failed: {reason}")
        return {"status": "NOT VERIFIED", "reason": reason}


def optical_transform_snapshot():
    # CAMERA_LINK_TO_OPTICAL_BASIS maps parent-frame coordinates into child
    # coordinates. A TF pose stores the inverse: child axes expressed in parent.
    return {
        "parent_frame": "camera_link",
        "child_frame": "camera_color_optical_frame",
        "translation_m": [0.0, 0.0, 0.0],
        "quaternion_xyzw": list(
            quaternion_xyzw_from_rotation(transpose(CAMERA_LINK_TO_OPTICAL_BASIS))
        ),
    }


def authoritative_box_bounds(center, dimensions):
    """Return world-space bounds for an axis-aligned authored box."""
    if len(center) != 3 or len(dimensions) != 3:
        raise ValueError("box center and dimensions must have three components")
    if any(float(value) <= 0.0 for value in dimensions):
        raise ValueError("box dimensions must be positive")
    half = [float(value) / 2.0 for value in dimensions]
    return (
        tuple(float(center[index]) - half[index] for index in range(3)),
        tuple(float(center[index]) + half[index] for index in range(3)),
    )


def inset_top_surface_points(minimum, maximum):
    """Return a centre and two inset off-axis points from USD world-space bounds."""
    x0, y0, z0 = minimum
    x1, y1, z1 = maximum
    return {
        "top_center": [(x0 + x1) / 2.0, (y0 + y1) / 2.0, z1],
        "top_off_axis_a": [x0 * 0.75 + x1 * 0.25, y0 * 0.75 + y1 * 0.25, z1],
        "top_off_axis_b": [x0 * 0.25 + x1 * 0.75, y0 * 0.25 + y1 * 0.75, z1],
    }


def inspect_stage(output_path, reference_prim_paths=None):
    """Inspect the composed stage without defining, changing, or saving any prim."""
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no USD stage is open")
    camera = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera.IsValid():
        raise RuntimeError(f"camera prim not found: {CAMERA_PATH}")
    xform_cache = UsdGeom.XformCache()
    transform = xform_cache.GetLocalToWorldTransform(camera)
    # Gf.Matrix4d is row-vector based; transpose yields conventional W_R_usd.
    usd_rotation = tuple(
        tuple(float(transform[j][i]) for j in range(3)) for i in range(3)
    )
    if not is_rigid_rotation(usd_rotation):
        raise RuntimeError(
            "composed Camera_Sensor transform is non-rigid; refusing TF snapshot"
        )
    link_rotation, optical_rotation = ros_camera_rotations(usd_rotation)
    reference_geometry = []
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for path in reference_prim_paths or [DEFAULT_REFERENCE_PRIM]:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"reference prim not found: {path}")
        bounds = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        bbox_minimum = [float(value) for value in bounds.GetMin()]
        bbox_maximum = [float(value) for value in bounds.GetMax()]
        geometry_attribute = prim.GetAttribute("sf_twin:geometry_dimensions_m")
        geometry_dimensions = (
            geometry_attribute.Get() if geometry_attribute.IsValid() else None
        )
        if geometry_dimensions is not None:
            minimum, maximum = authoritative_box_bounds(
                [
                    (bbox_minimum[index] + bbox_maximum[index]) / 2.0
                    for index in range(3)
                ],
                geometry_dimensions,
            )
            bounds_source = (
                "sf_twin:geometry_dimensions_m centered on composed prim bounds"
            )
        else:
            minimum, maximum = bbox_minimum, bbox_maximum
            bounds_source = "UsdGeom.BBoxCache.ComputeWorldBound"
        reference_geometry.append(
            {
                "prim_path": path,
                "world_bounds_m": {"min": minimum, "max": maximum},
                "world_surface_points_m": inset_top_surface_points(minimum, maximum),
                "bounds_source": bounds_source,
            }
        )
    origin = transform.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    direction = lambda x, y, z: [
        float(value) for value in transform.TransformDir(Gf.Vec3d(x, y, z))
    ]
    axes = {
        "local_plus_x_world": direction(1, 0, 0),
        "local_plus_y_world": direction(0, 1, 0),
        "local_plus_z_world": direction(0, 0, 1),
        "local_minus_z_forward_world": direction(0, 0, -1),
    }
    for geometry in reference_geometry:
        for name, point in geometry["world_surface_points_m"].items():
            delta = [point[index] - float(origin[index]) for index in range(3)]
            geometry.setdefault("forward_alignment", {})[name] = {
                "world_delta": delta,
                "forward_dot": normalized_dot(
                    axes["local_minus_z_forward_world"], delta
                ),
            }
    snapshot = {
        "schema_version": 1,
        "camera_prim": CAMERA_PATH,
        "stage_meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "world_to_usd_camera": {
            "parent_frame": "world",
            "child_frame": "usd_camera_frame",
            "translation_m": [float(transform[3][i]) for i in range(3)],
            "quaternion_xyzw": list(quaternion_xyzw_from_rotation(usd_rotation)),
        },
        "usd_camera_to_ros_optical_point_coordinate_basis": [
            list(row) for row in USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS
        ],
        "world_to_optical": {
            "parent_frame": "world",
            "child_frame": "camera_color_optical_frame",
            "translation_m": [float(transform[3][i]) for i in range(3)],
            "quaternion_xyzw": list(quaternion_xyzw_from_rotation(optical_rotation)),
        },
        "usd_composed_diagnostics": {
            "raw_matrix_row_major": [
                [float(transform[row][column]) for column in range(4)]
                for row in range(4)
            ],
            "translation_row_3": [float(transform[3][index]) for index in range(3)],
            "translation_column_3": [float(transform[index][3]) for index in range(3)],
            "camera_origin_world": [float(value) for value in origin],
            "basis_directions_world": axes,
        },
        "render_product_bindings": render_product_camera_bindings(stage, CAMERA_PATH),
        "d455_color_camera_pose_comparison": d455_color_camera_pose_comparison(
            stage, camera, xform_cache
        ),
        "depth_helper_runtime": depth_helper_runtime_snapshot(stage),
        "world_to_camera_link": {
            "parent_frame": "world",
            "child_frame": "camera_link",
            "translation_m": [float(transform[3][i]) for i in range(3)],
            "quaternion_xyzw": list(quaternion_xyzw_from_rotation(link_rotation)),
        },
        "camera_link_to_optical": optical_transform_snapshot(),
        "projection": {
            name: camera.GetAttribute(name).Get()
            for name in (
                "focalLength",
                "horizontalAperture",
                "verticalAperture",
                "clippingRange",
            )
        },
        "reference_geometry": reference_geometry,
        "collision_scene_query": collision_scene_query_snapshot(
            transform, reference_geometry
        ),
    }
    output = Path(output_path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(snapshot, indent=2, default=list) + "\n")
    print(f"Camera snapshot written: {output.resolve()}")
    return snapshot


def script_editor_output_path(
    environ, script_path=None, default_filename="camera_snapshot.json"
):
    """Return an explicit override or the stable repo-local snapshot path."""
    return str(
        output_path(
            environ.get("SFTWIN_CAMERA_SNAPSHOT_PATH"),
            "camera_inspection",
            default_filename,
            script_path if script_path is not None else __file__,
            environ,
        )
    )


def script_editor_reference_prims(environ):
    raw = environ.get("SFTWIN_CAMERA_REFERENCE_PRIMS", DEFAULT_REFERENCE_PRIM)
    return [path.strip() for path in raw.split(",") if path.strip()]


def run_cli(argv, inspector=inspect_stage):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    return inspector(args.output)


def run_entrypoint(
    argv=None,
    script_path=None,
    environ=None,
    inspector=inspect_stage,
    output_filename="camera_snapshot.json",
):
    """Choose strict CLI parsing or the Script Editor-safe environment entrypoint."""
    effective_path = script_path if script_path is not None else __file__
    if is_script_editor_path(effective_path):
        environment = os.environ if environ is None else environ
        return inspector(
            script_editor_output_path(environment, effective_path, output_filename),
            script_editor_reference_prims(environment),
        )
    return run_cli(argv, inspector)


if __name__ == "__main__":
    print(json.dumps(run_entrypoint(), indent=2, default=list))
