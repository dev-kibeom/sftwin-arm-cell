from pathlib import Path

from camera_tooling import inspection_math
from shared.runtime_articulation_probe import candidate_paths
from isaac_runtime.robot_binding import RobotBinding


CELL_ROOT = Path(__file__).resolve().parents[2]


def test_inspection_math_imports_without_isaac_runtime():
    assert inspection_math.usd_camera_to_optical((1.0, 2.0, -3.0)) == (1.0, -2.0, 3.0)


def test_runtime_articulation_candidates_follow_selected_binding():
    binding = RobotBinding('/World/Other/Robot', '/World/Other/Robot/joints/root',
                           '/World/Other/Robot/links/base', (), (), {})
    assert candidate_paths(binding) == (
        binding.robot_path, binding.articulation_root_path, binding.base_link_path,
    )


def test_camera_entrypoint_exports_inspection_contract():
    import importlib.util

    wrapper = CELL_ROOT / "camera_tooling/entrypoints/inspect_camera.py"
    spec = importlib.util.spec_from_file_location(
        "inspect_camera_compatibility", wrapper
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.inspect_stage)
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


def test_articulation_probe_imports_from_standalone_cell_tooling():
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, '-I', '-c',
         'import sys; sys.path.insert(0, sys.argv[1]); '
         'from shared.runtime_articulation_probe import candidate_paths', str(CELL_ROOT)],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
