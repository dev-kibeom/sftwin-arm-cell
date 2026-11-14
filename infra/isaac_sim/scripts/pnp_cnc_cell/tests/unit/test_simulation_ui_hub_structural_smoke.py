from pathlib import Path


def test_isaac_extension_has_loadable_entrypoint():
    repository = Path(__file__).resolve().parents[6]
    root = repository / "infra/isaac_sim/extensions/arm_cell.simulation_ui_hub"
    import sys

    sys.path.insert(0, str(root / "python"))
    from arm_cell_simulation_ui_hub.extension import extension_class

    entrypoint = extension_class()
    assert isinstance(entrypoint, type)
    assert isinstance(entrypoint(), entrypoint)
