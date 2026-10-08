"""Focused checks for role sequencing and persistent Script Editor globals."""

import importlib.util
from pathlib import Path


CELL_ROOT = Path(__file__).parents[2]
WORKFLOW_PATH = CELL_ROOT / "shared/runtime_workflow.py"


def _load_workflow():
    spec = importlib.util.spec_from_file_location(
        "runtime_workflow_test", WORKFLOW_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.module_root_from_project_root = lambda _root: CELL_ROOT
    return module


def test_roles_sequence_existing_build_and_runtime_steps(tmp_path, monkeypatch):
    import os
    from shared import script_editor_bootstrap

    workflow = _load_workflow()
    project_root = tmp_path / "checkout"
    module_root = project_root / "infra/isaac_sim/scripts/m0609_cell"
    steps = module_root / "runtime_steps"
    steps.mkdir(parents=True)
    (project_root / "ros2_ws/src/arm_cell").mkdir(parents=True)
    workflow.module_root_from_project_root = lambda _root: module_root
    monkeypatch.setattr(
        script_editor_bootstrap,
        "module_root_from_project_root",
        lambda _root: module_root,
    )
    namespace = {"events": []}
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(project_root))
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    monkeypatch.delenv("SFTWIN_PNP_PROFILE", raising=False)
    monkeypatch.delenv("SFTWIN_RUNTIME_MODE", raising=False)
    for role in workflow.ROLES:
        namespace["events"] = []
        for index, filename in enumerate(workflow.ROLES[role]):
            (steps / filename).write_text(
                f"events.append(({role!r}, {index}))\n", encoding="utf-8"
            )
        workflow.run_role(role, namespace)
        assert len(namespace["events"]) == len(workflow.ROLES[role])
        assert namespace["SFTWIN_RUNTIME_ROLE"] == role
        assert os.environ["SFTWIN_PROJECT_ROOT"] == str(project_root)
        if role == "pre_build":
            assert "ROS_DOMAIN_ID" not in os.environ
            assert "SFTWIN_PNP_PROFILE" not in os.environ
            assert "SFTWIN_RUNTIME_MODE" not in os.environ
        if role == "after_play":
            assert os.environ["SFTWIN_RUNTIME_MODE"] == "production"
