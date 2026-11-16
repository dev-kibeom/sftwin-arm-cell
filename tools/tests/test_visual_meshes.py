"""The asset authoring tool consumes shapes/materials, not robot identities."""

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def tool():
    spec = importlib.util.spec_from_file_location(
        "visual_meshes", ROOT / "tools/visual_meshes.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_unrelated_model_names_materials_and_resolution(tmp_path):
    recipe = {
        "segments": 12,
        "materials": {
            "blue_paint": {"diffuse": [0.1, 0.3, 0.8, 1], "shininess": 10},
            "lamp": {"diffuse": [0.8, 0.2, 0.1, 1], "emission": [0.4, 0.1, 0.05, 1]},
        },
        "links": {
            "housing": [
                {
                    "shape": "body",
                    "center": [0, 0, 0],
                    "axis": [0, 0, 1],
                    "length": 0.2,
                    "radius": 0.05,
                    "material": "blue_paint",
                }
            ],
            "indicator": [
                {
                    "shape": "ring",
                    "center": [0, 0, 0.1],
                    "axis": [0, 0, 1],
                    "length": 0.01,
                    "outer": 0.05,
                    "inner": 0.04,
                    "material": "lamp",
                }
            ],
        },
        "revision": {"macro": "fixture_visual_revision", "label": "Fixture visuals"},
    }
    source = tmp_path / "appearance.json"
    source.write_text(json.dumps(recipe))
    output = tmp_path / "output"
    tool().generate(source, output)
    assert {p.name for p in output.glob("*.dae")} == {"housing.dae", "indicator.dae"}
    mesh = ET.parse(output / "housing.dae")
    assert mesh.find("c:asset/c:unit", NS).get("meter") == "1"
    assert {
        e.get("id") for e in mesh.findall("c:library_materials/c:material", NS)
    } == {"blue_paint", "lamp"}
    assert (
        ET.parse(output / "revision.xacro")
        .find("{http://www.ros.org/wiki/xacro}macro")
        .get("name")
        == "fixture_visual_revision"
    )
    before = (output / "housing.dae").read_bytes()
    recipe["segments"] = 24
    source.write_text(json.dumps(recipe))
    tool().generate(source, output)
    assert (output / "housing.dae").read_bytes() != before


def test_hcr14_recipe_preserves_canonical_visual_assets(tmp_path):
    recipe = ROOT / "assets/robots/hcr14/visuals.json"
    module = tool()
    module.generate(recipe, tmp_path)
    names = [f"{name}.dae" for name in json.loads(recipe.read_text())["links"]]
    for name in [*names, "revision.xacro"]:
        assert (tmp_path / name).read_bytes() == (
            recipe.parent / "visual" / name
        ).read_bytes()


def test_sharp_body_and_material_ids_do_not_depend_on_exporter_names(tmp_path):
    recipe = {
        "materials": {
            name: {"diffuse": [0.5, 0.5, 0.5, 1]}
            for name in ["Scene", "part0", "paint", "paint-fx"]
        },
        "links": {
            "component": [
                {
                    "shape": "body",
                    "center": [0, 0, 0],
                    "axis": [0, 0, 1],
                    "length": 0.2,
                    "radius": 0.05,
                    "edge": 0,
                    "material": "Scene",
                },
                {
                    "shape": "body",
                    "center": [0, 0, 0.2],
                    "axis": [0, 0, 1],
                    "length": 0.2,
                    "radius": 0.05,
                    "material": "part0",
                },
            ]
        },
    }
    source = tmp_path / "appearance.json"
    source.write_text(json.dumps(recipe))
    output = tool().generate(source)
    mesh = ET.parse(output / "component.dae")
    identifiers = [e.get("id") for e in mesh.iter() if e.get("id")]
    assert len(identifiers) == len(set(identifiers))
    for binding in mesh.findall(".//c:instance_material", NS):
        assert binding.get("target")[1:] in identifiers
