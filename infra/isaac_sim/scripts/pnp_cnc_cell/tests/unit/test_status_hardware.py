"""Status hardware must have continuous physical support."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[6]
SOURCE = ROOT / "infra/isaac_sim/scripts/pnp_cnc_cell/scene_builder/cell_metadata.py"
spec = importlib.util.spec_from_file_location("status_metadata", SOURCE)
metadata = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metadata)


def test_estop_pedestal_reaches_floor_and_supports_box():
    boxes = {}
    context = SimpleNamespace(
        stage=object(), cell_root="/World/Cell",
        colors=dict.fromkeys(("charcoal", "safety", "red", "amber", "green", "black")),
        materials=dict.fromkeys(("mat_frame", "mat_safety", "mat_red", "mat_amber", "mat_green", "mat_black")),
        fixed_box=lambda path, name, pos, size, *args: boxes.update({path: (pos, size)}),
        create_cylinder=lambda *args, **kwargs: None,
        create_sphere=lambda *args: None,
        set_custom_data=lambda *args: None,
    )
    metadata.build_status_hardware(context)
    pos, size = boxes["/World/Cell/Status/EStop/Pedestal"]
    floor = next(obj for obj in json.loads(
        (ROOT / "config/pnp_cnc_static_environment.json").read_text()
    )["objects"] if obj["id"] == "factory_floor")
    floor_top = floor["pose"]["position_m"][2] + floor["geometry"]["dimensions_m"][2] / 2
    assert pos[2] - size[2] / 2 == pytest.approx(floor_top)
    box_pos, box_size = boxes["/World/Cell/Status/EStop/Box"]
    assert pos[:2] == box_pos[:2]
    assert pos[2] + size[2] / 2 >= box_pos[2] - box_size[2] / 2
