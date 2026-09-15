from pathlib import Path

import pytest

from camera_tooling import stage_inspector as camera


def test_optical_basis_is_rigid_and_matches_declared_axes():
    assert camera.is_rigid_rotation(camera.CAMERA_LINK_TO_OPTICAL_BASIS)
    assert camera.optical_transform_snapshot()["translation_m"] == [0.0, 0.0, 0.0]


def test_optical_basis_maps_camera_link_forward_to_optical_forward():
    basis = camera.CAMERA_LINK_TO_OPTICAL_BASIS
    assert [
        sum(basis[row][column] * [1.0, 0.0, 0.0][column] for column in range(3))
        for row in range(3)
    ] == [0.0, 0.0, 1.0]


def test_optical_tf_pose_is_inverse_of_point_coordinate_mapping():
    point_mapping = camera.CAMERA_LINK_TO_OPTICAL_BASIS
    pose = camera.transpose(point_mapping)
    assert tuple(
        tuple(
            sum(point_mapping[row][index] * pose[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    ) == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert [
        sum(point_mapping[row][column] * [1.0, 0.0, 0.0][column] for column in range(3))
        for row in range(3)
    ] == [0.0, 0.0, 1.0]
    assert [
        sum(point_mapping[row][column] * [0.0, 1.0, 0.0][column] for column in range(3))
        for row in range(3)
    ] == [-1.0, 0.0, 0.0]
    assert [
        sum(point_mapping[row][column] * [0.0, 0.0, 1.0][column] for column in range(3))
        for row in range(3)
    ] == [0.0, -1.0, 0.0]


def test_nonidentity_pose_reconstructs_usd_derived_optical_pose_from_link_and_optical():
    world_from_usd_camera = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    world_from_link, derived_optical = camera.ros_camera_rotations(
        world_from_usd_camera
    )
    link_from_optical = camera.transpose(camera.CAMERA_LINK_TO_OPTICAL_BASIS)
    assert derived_optical == camera.multiply(
        world_from_usd_camera, camera.USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS
    )
    assert camera.multiply(world_from_link, link_from_optical) == derived_optical


def test_nonidentity_usd_direct_point_conversion_matches_camera_link_tf_chain():
    world_from_usd_camera = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    world_from_link, _ = camera.ros_camera_rotations(world_from_usd_camera)
    usd_point = (0.2, -0.3, -2.0)
    world_point = camera.matrix_vector(world_from_usd_camera, usd_point)
    direct_optical = camera.usd_camera_to_optical(
        camera.matrix_vector(camera.transpose(world_from_usd_camera), world_point)
    )
    link_point = camera.matrix_vector(camera.transpose(world_from_link), world_point)
    chain_optical = camera.matrix_vector(
        camera.CAMERA_LINK_TO_OPTICAL_BASIS, link_point
    )
    assert direct_optical == pytest.approx((0.2, 0.3, 2.0))
    assert chain_optical == pytest.approx(direct_optical)


def test_pose_difference_reports_translation_and_rotation_in_reference_camera_axes():
    reference_rotation = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    candidate_rotation = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    difference = camera.pose_difference(
        reference_rotation, (1.0, 2.0, 3.0), candidate_rotation, (2.0, 2.0, 3.5)
    )
    assert difference["translation_delta_world_m"] == [1.0, 0.0, 0.5]
    assert difference["translation_delta_in_reference_local_m"] == pytest.approx(
        [0.0, -1.0, 0.5]
    )
    assert difference["rotation_delta_rad"] == pytest.approx(1.5707963267948966)
    assert difference["relative_rotation_matrix"] == [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def test_pose_difference_rejects_nonrigid_camera_pose():
    with pytest.raises(ValueError, match="scale"):
        camera.pose_difference(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
            ((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            (0.0, 0.0, 0.0),
        )


def test_affine_diagnostics_preserve_nonrigid_scale_and_shear_evidence():
    diagnostic = camera.affine_3x3_diagnostics(
        ((2.0, 0.5, 0.0), (0.0, 3.0, 0.0), (0.0, 0.0, 4.0))
    )
    assert diagnostic["upper_left_3x3"] == [
        [2.0, 0.5, 0.0],
        [0.0, 3.0, 0.0],
        [0.0, 0.0, 4.0],
    ]
    assert diagnostic["row_norms"] == pytest.approx([2.0615528128, 3.0, 4.0])
    assert diagnostic["column_norms"] == pytest.approx([2.0, 3.0413812651, 4.0])
    assert diagnostic["determinant"] == pytest.approx(24.0)
    assert diagnostic["column_dot_products"][0][1] == pytest.approx(1.0)
    assert diagnostic["is_rigid_rotation"] is False
    assert diagnostic["scale_shear_decomposition"]["status"] == "NOT VERIFIED"


def test_usd_camera_axes_convert_to_ros_optical_axes():
    assert camera.usd_camera_to_optical((0.0, 0.0, -2.0)) == (0.0, 0.0, 2.0)
    assert camera.usd_camera_to_optical((2.0, 0.0, 0.0)) == (2.0, 0.0, 0.0)
    assert camera.usd_camera_to_optical((0.0, 2.0, 0.0)) == (0.0, -2.0, 0.0)


def test_usd_direct_optical_conversion_is_a_rigid_basis_change():
    assert camera.is_rigid_rotation(camera.USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS)


def test_nonidentity_basis_alignment_distinguishes_forward_from_reverse():
    forward = (0.0, 0.0, -1.0)
    assert camera.normalized_dot(forward, (0.0, 0.0, -2.0)) == pytest.approx(1.0)
    assert camera.normalized_dot(forward, (0.0, 0.0, 2.0)) == pytest.approx(-1.0)


def test_reference_ray_direction_is_normalized_and_retains_distance():
    direction, distance = camera.ray_direction((1.0, 2.0, 3.0), (1.0, 5.0, 7.0))
    assert direction == pytest.approx((0.0, 0.6, 0.8))
    assert distance == pytest.approx(5.0)


def test_reference_ray_direction_rejects_camera_origin_as_a_surface_point():
    with pytest.raises(ValueError, match="camera origin"):
        camera.ray_direction((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))


def test_render_product_binding_report_has_explicit_camera_match_flag():
    record = {
        "render_product": "/Render/Product",
        "camera_targets": [camera.CAMERA_PATH],
        "uses_camera_sensor": True,
    }
    assert record["uses_camera_sensor"] is (
        camera.CAMERA_PATH in record["camera_targets"]
    )


def test_graph_connection_descriptor_uses_the_attribute_owning_node():
    class Node:
        def get_prim_path(self):
            return "/Graph/CreateRenderProduct"

    class Attribute:
        def get_node(self):
            return Node()

        def get_name(self):
            return "outputs:renderProductPath"

    descriptor = camera.graph_connection_descriptor(Attribute())
    assert descriptor["owning_node_path"] == "/Graph/CreateRenderProduct"
    assert descriptor["attribute_name"] == "outputs:renderProductPath"
    assert "get_node" in descriptor["available_api"]


def test_graph_connection_descriptor_rejects_node_api_on_an_attribute():
    with pytest.raises(RuntimeError, match="get_node"):
        camera.graph_connection_descriptor(object())


def test_world_point_in_front_of_usd_camera_projects_in_bounds():
    usd_local = (0.0, 0.0, -1.0)
    optical = camera.usd_camera_to_optical(usd_local)
    assert optical[2] > 0.0
    assert (
        1465.9985736982494 * optical[0] / optical[2] + 640.0,
        1465.9985736982494 * optical[1] / optical[2] + 360.0,
    ) == (640.0, 360.0)


def test_reference_surface_exports_center_and_off_axis_points():
    points = camera.inset_top_surface_points((0.0, 10.0, 2.0), (8.0, 14.0, 3.0))
    assert points["top_center"] == [4.0, 12.0, 3.0]
    assert points["top_off_axis_a"] != points["top_center"]


def test_non_rigid_transform_is_rejected():
    assert not camera.is_rigid_rotation(
        ((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    )
    with pytest.raises(ValueError):
        camera.quaternion_xyzw_from_rotation(
            ((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        )


def test_script_editor_uses_explicit_output_without_parsing_kit_arguments():
    observed = []
    snapshot = camera.run_entrypoint(
        argv=["--kit-argument", "unexpected"],
        script_path="/tmp/carb/script_1789055439.py",
        environ={"SFTWIN_CAMERA_SNAPSHOT_PATH": "/tmp/camera_snapshot.json"},
        inspector=lambda path, *_: observed.append(path) or {"path": path},
    )
    assert observed == ["/tmp/camera_snapshot.json"]
    assert snapshot["path"] == "/tmp/camera_snapshot.json"


def test_script_editor_uses_stable_repo_local_output_without_override():
    output = camera.script_editor_output_path(
        {"SFTWIN_PROJECT_ROOT": str(Path(__file__).resolve().parents[6])},
        "/tmp/carb/script_1789055439.py",
    )
    assert output.endswith(
        ".local_artifacts/m0609_cell/camera_inspection/camera_snapshot.json"
    )


def test_script_editor_snapshot_override_is_preserved():
    assert (
        camera.script_editor_output_path(
            {"SFTWIN_CAMERA_SNAPSHOT_PATH": "/tmp/operator_snapshot.json"},
            "/workspace/inspect_camera.py",
        )
        == "/tmp/operator_snapshot.json"
    )


def test_standalone_cli_remains_strict_about_unknown_arguments():
    with pytest.raises(SystemExit) as error:
        camera.run_entrypoint(
            argv=["--output", "/tmp/snapshot.json", "--unknown"],
            script_path="/workspace/inspect_camera.py",
            inspector=lambda _: None,
        )
    assert error.value.code == 2


def test_script_editor_output_precedence_is_environment_then_wrapper_then_library_default():
    project_root = str(Path(__file__).resolve().parents[6])
    script_path = "/tmp/carb/script_1789055439.py"
    assert (
        camera.script_editor_output_path(
            {
                "SFTWIN_PROJECT_ROOT": project_root,
                "SFTWIN_CAMERA_SNAPSHOT_PATH": "/tmp/env.json",
            },
            script_path,
            "wrapper.json",
        )
        == "/tmp/env.json"
    )
    assert camera.script_editor_output_path(
        {"SFTWIN_PROJECT_ROOT": project_root}, script_path, "wrapper.json"
    ).endswith("camera_inspection/wrapper.json")
    assert camera.script_editor_output_path(
        {"SFTWIN_PROJECT_ROOT": project_root}, script_path
    ).endswith("camera_inspection/camera_snapshot.json")


def test_standalone_cli_output_precedes_script_editor_filename_default():
    observed = []
    camera.run_entrypoint(
        argv=["--output", "/tmp/cli.json"],
        script_path="/workspace/inspect_camera.py",
        inspector=lambda path: observed.append(path),
        output_filename="wrapper.json",
    )
    assert observed == ["/tmp/cli.json"]
