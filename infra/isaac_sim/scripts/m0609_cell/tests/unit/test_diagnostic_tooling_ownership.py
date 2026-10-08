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


def test_camera_entrypoint_exports_inspection_contract():
    import importlib.util

    wrapper = CELL_ROOT / "camera_tooling/entrypoints/inspect_camera.py"
    spec = importlib.util.spec_from_file_location(
        "inspect_camera_compatibility", wrapper
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.CAMERA_PATH == inspection_math.CAMERA_PATH
    assert module.usd_camera_to_optical((0.0, 0.0, -1.0)) == (0.0, 0.0, 1.0)


def test_renderer_entrypoint_exports_canonical_authoring():
    import importlib.util

    wrapper = CELL_ROOT / "camera_tooling/entrypoints/renderer_depth_calibration.py"
    spec = importlib.util.spec_from_file_location(
        "renderer_depth_calibration_compatibility", wrapper
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.author_targets)
    assert module.ROOT == "/World/SF_Twin_Acceptance/RendererDepth"
