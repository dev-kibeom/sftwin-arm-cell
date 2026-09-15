from pathlib import Path

from camera_tooling import inspection_math
from shared.runtime_articulation_probe import CANDIDATE_PATHS


CELL_ROOT = Path(__file__).resolve().parents[2]


def test_inspection_math_imports_without_isaac_runtime():
    assert inspection_math.usd_camera_to_optical((1.0, 2.0, -3.0)) == (1.0, -2.0, 3.0)


def test_runtime_articulation_candidates_preserve_legacy_probe_order():
    assert CANDIDATE_PATHS == (
        "/m0609_robotiq_2f85",
        "/m0609_robotiq_2f85/root_joint",
        "/m0609_robotiq_2f85/base_link",
        "/m0609_robotiq_2f85/base",
        "/m0609_robotiq_2f85/link_1",
    )


def test_manual_probe_is_not_a_production_runtime_dependency():
    session_source = (CELL_ROOT / "gripper_runtime/session.py").read_text()
    assert "manual.gripper_attachment_probe" not in session_source
    assert (
        "omni"
        not in (CELL_ROOT / "manual/gripper_attachment_probe.py")
        .read_text()
        .split("def run_probe", 1)[0]
    )


def test_camera_compatibility_entrypoint_reexports_inspection_contract():
    import importlib.util

    wrapper = CELL_ROOT / "inspect_camera.py"
    spec = importlib.util.spec_from_file_location(
        "inspect_camera_compatibility", wrapper
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.CAMERA_PATH == inspection_math.CAMERA_PATH
    assert module.usd_camera_to_optical((0.0, 0.0, -1.0)) == (0.0, 0.0, 1.0)


def test_renderer_compatibility_entrypoint_reexports_canonical_authoring():
    import importlib.util

    wrapper = CELL_ROOT / "renderer_depth_calibration.py"
    spec = importlib.util.spec_from_file_location(
        "renderer_depth_calibration_compatibility", wrapper
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.author_targets)
    assert module.ROOT == "/World/SF_Twin_Acceptance/RendererDepth"
