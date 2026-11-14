"""Remove importer-authored visual references for geometry-free URDF links."""

import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Sdf


def canonical_visual_requirements(urdf_path: Path) -> tuple[str, dict[str, bool]]:
    robot = ET.parse(urdf_path).getroot()
    robot_name = robot.get("name")
    if not robot_name:
        raise RuntimeError("generated URDF has no robot name")
    requirements = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name:
            raise RuntimeError("generated URDF has an unnamed link")
        requirements[name] = bool(link.findall("visual"))
    return robot_name, requirements


def cleanup_dangling_visual_references(
    layer: Sdf.Layer, robot_path: str, visual_requirements: dict[str, bool]
) -> list[str]:
    cleaned = []
    for link, requires_visual in visual_requirements.items():
        target_path = f"/visuals/{link}"
        target = layer.GetPrimAtPath(target_path)
        if target is not None:
            continue
        visual = layer.GetPrimAtPath(f"{robot_path}/{link}/visuals")
        if requires_visual:
            raise RuntimeError(
                f"generated USD is missing visual geometry for required link '{link}'"
            )
        if visual is None:
            continue
        references = visual.referenceList.prependedItems
        remaining = [
            reference
            for reference in references
            if str(reference.primPath) != target_path
        ]
        if len(remaining) != len(references):
            visual.referenceList.prependedItems = remaining
            cleaned.append(link)
    return cleaned


def cleanup_generated_base_layer(urdf_path: Path, base_layer_path: Path) -> list[str]:
    layer = Sdf.Layer.FindOrOpen(str(base_layer_path))
    if layer is None:
        raise RuntimeError(f"generated base USD layer cannot be opened: {base_layer_path}")
    robot_name, requirements = canonical_visual_requirements(urdf_path)
    cleaned = cleanup_dangling_visual_references(layer, f"/{robot_name}", requirements)
    if cleaned:
        if not layer.Save():
            raise RuntimeError(f"generated base USD layer cannot be saved: {base_layer_path}")
    return cleaned
