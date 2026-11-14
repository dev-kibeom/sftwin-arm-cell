"""Camera gantry centering and AMR interface attachment contracts."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[6]
SOURCE = ROOT / "infra/isaac_sim/scripts/pnp_cnc_cell/scene_builder/environment.py"
spec = importlib.util.spec_from_file_location("rig_environment", SOURCE)
environment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(environment)


def geometry():
    return {obj["id"]: obj for obj in json.loads(
        (ROOT / "config/pnp_cnc_static_environment.json").read_text()
    )["objects"]}


def test_camera_gantry_is_centered_on_camera_mount():
    objects = geometry()
    left, right, beam, mount = [objects[key] for key in (
        "camera_rig_post_left", "camera_rig_post_right",
        "camera_rig_top_beam", "camera_boom",
    )]
    center = mount["pose"]["position_m"][0]
    lx, rx = [obj["pose"]["position_m"][0] for obj in (left, right)]
    assert (lx + rx) / 2 == pytest.approx(center)
    assert beam["pose"]["position_m"][0] == pytest.approx(center)
    for post in (left, right):
        assert abs(post["pose"]["position_m"][0] - center) + post["geometry"]["dimensions_m"][0] / 2 <= beam["geometry"]["dimensions_m"][0] / 2


@pytest.mark.parametrize("shift", [0.0, 0.25])
def test_interface_posts_and_leds_follow_camera_posts(shift):
    objects = geometry()
    for key in ("camera_rig_post_left", "camera_rig_post_right"):
        objects[key]["pose"]["position_m"][0] += shift
    boxes, leds = {}, {}
    context = SimpleNamespace(
        stage=object(), cell_root="/World/Cell", static_geometry=objects,
        colors=dict.fromkeys(("charcoal", "green", "cyan")),
        materials=dict.fromkeys(("mat_frame", "mat_green", "mat_cyan")),
        fixed_box=lambda path, name, pos, size, *args: boxes.update({path: (pos, size)}),
        create_sphere=lambda stage, path, radius, pos, *args: leds.update({path: pos}),
        set_custom_data=lambda *args: None,
    )
    environment.build_amr_interface(context)
    for side in ("Left", "Right"):
        post = objects[f"camera_rig_post_{side.lower()}"]
        pos, size = boxes[f"/World/Cell/Safety/AMRInterface/{side}Post"]
        rig_pos = post["pose"]["position_m"]
        rig_size = post["geometry"]["dimensions_m"]
        assert abs(pos[0] - rig_pos[0]) <= (size[0] + rig_size[0]) / 2
        assert abs(pos[1] - rig_pos[1]) <= (size[1] + rig_size[1]) / 2
        assert leds[f"/World/Cell/Safety/AMRInterface/{side}LED"][0] == pos[0]
    left = boxes["/World/Cell/Safety/AMRInterface/LeftPost"][0][0]
    right = boxes["/World/Cell/Safety/AMRInterface/RightPost"][0][0]
    pos, size = boxes["/World/Cell/Safety/AMRInterface/BeamIndicator"]
    assert pos[0] == pytest.approx((left + right) / 2)
    assert size[0] == pytest.approx(right - left - 0.12)
