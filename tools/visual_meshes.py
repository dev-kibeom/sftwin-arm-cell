#!/usr/bin/env python3
"""Generate COLLADA visual meshes from reusable primitive/profile recipes.

Python stdlib only. Geometry is visual authoring data, independent of URDF
joints, inertias, collision geometry, robot names and simulator importers.
"""

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", NS)


def add(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, f"{{{NS}}}{tag}", attributes)
    child.text = text
    return child


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(a):
    length = math.sqrt(sum(v * v for v in a))
    if not length:
        raise ValueError("axis must be nonzero")
    return tuple(v / length for v in a)


class Shell:
    def __init__(self, segments=64):
        if not isinstance(segments, int) or isinstance(segments, bool) or segments < 3:
            raise ValueError("segments must be an integer of at least 3")
        self.segments = segments
        self.parts = []

    def lathe(self, center, axis, profile, material):
        """Revolve an axial/radial outline with smooth vertex normals."""
        axis = unit(axis)
        u = unit(cross((0, 1, 0) if abs(axis[2]) > 0.9 else (0, 0, 1), axis))
        v = cross(axis, u)
        points, normals, faces = [], [], []
        for row, (z, radius) in enumerate(profile):
            prev = profile[max(0, row - 1)]
            following = profile[min(len(profile) - 1, row + 1)]
            dz, dr = following[0] - prev[0], following[1] - prev[1]
            normal_scale = math.hypot(dz, dr)
            for column in range(self.segments):
                angle = 2 * math.pi * column / self.segments
                radial = tuple(
                    math.cos(angle) * a + math.sin(angle) * b for a, b in zip(u, v)
                )
                points.append(
                    tuple(
                        c + z * a + radius * r for c, a, r in zip(center, axis, radial)
                    )
                )
                normals.append(
                    tuple(
                        (dz * r - dr * a) / normal_scale for r, a in zip(radial, axis)
                    )
                )
        for row in range(len(profile) - 1):
            for column in range(self.segments):
                a = row * self.segments + column
                b = row * self.segments + (column + 1) % self.segments
                for face in (
                    (a, b, b + self.segments),
                    (a, b + self.segments, a + self.segments),
                ):
                    p, q, r = (points[i] for i in face)
                    area = cross(
                        tuple(y - x for x, y in zip(p, q)),
                        tuple(y - x for x, y in zip(p, r)),
                    )
                    if sum(x * x for x in area) > 1e-22:
                        faces.append(face)
        self.parts.append((points, normals, faces, material))

    def body(self, center, axis, length, radius, material, edge=0.006):
        if length <= 0 or radius <= 0 or edge < 0:
            raise ValueError("body dimensions must be positive and edge nonnegative")
        edge = min(edge, length / 4, radius / 4)
        half = length / 2
        if edge == 0:
            self.lathe(
                center,
                axis,
                [(-half, 0), (-half, radius), (half, radius), (half, 0)],
                material,
            )
            return
        self.lathe(
            center,
            axis,
            [
                (-half, 0),
                (-half, radius - edge),
                (-half + edge * 0.3, radius - edge * 0.3),
                (-half + edge, radius),
                (half - edge, radius),
                (half - edge * 0.3, radius - edge * 0.3),
                (half, radius - edge),
                (half, 0),
            ],
            material,
        )

    def ring(self, center, axis, length, outer, inner, material):
        h = length / 2
        self.lathe(
            center,
            axis,
            [(-h, inner), (-h, outer), (h, outer), (h, inner), (-h, inner)],
            material,
        )

    def bolts(
        self, center, axis, pitch_radius, count, material, size=0.004, head_length=0.002
    ):
        axis = unit(axis)
        u = unit(cross((0, 1, 0) if abs(axis[2]) > 0.9 else (0, 0, 1), axis))
        v = cross(axis, u)
        for i in range(count):
            angle = 2 * math.pi * (i + 0.5) / count
            p = tuple(
                c + pitch_radius * (math.cos(angle) * a + math.sin(angle) * b)
                for c, a, b in zip(center, u, v)
            )
            self.body(p, axis, head_length, size, material, edge=0.0005)

    def save(self, path, materials, metadata=None):
        root = ET.Element(f"{{{NS}}}COLLADA", version="1.4.1")
        metadata = metadata or {}
        asset = add(root, "asset")
        add(asset, "contributor")
        add(asset, "created", metadata.get("created", "1970-01-01T00:00:00Z"))
        add(
            asset,
            "modified",
            metadata.get("modified", metadata.get("created", "1970-01-01T00:00:00Z")),
        )
        add(asset, "unit", name="meter", meter="1")
        add(asset, "up_axis", "Z_UP")
        effects = add(root, "library_effects")
        material_library = add(root, "library_materials")
        geometries = add(root, "library_geometries")
        scene = add(add(root, "library_visual_scenes"), "visual_scene", id="Scene")
        for index, (points, normals, faces, material) in enumerate(self.parts):
            identity = f"part{index}"
            mesh = add(add(geometries, "geometry", id=identity), "mesh")
            for suffix, values in [("positions", points), ("normals", normals)]:
                sid = f"{identity}-{suffix}"
                source = add(mesh, "source", id=sid)
                add(
                    source,
                    "float_array",
                    " ".join(f"{v:.9g}" for vector in values for v in vector),
                    id=sid + "-array",
                    count=str(len(values) * 3),
                )
                accessor = add(
                    add(source, "technique_common"),
                    "accessor",
                    source="#" + sid + "-array",
                    count=str(len(values)),
                    stride="3",
                )
                for component in "XYZ":
                    add(accessor, "param", name=component, type="float")
            vertices = add(mesh, "vertices", id=identity + "-vertices")
            add(vertices, "input", semantic="POSITION", source=f"#{identity}-positions")
            triangles = add(mesh, "triangles", material=material, count=str(len(faces)))
            add(
                triangles,
                "input",
                semantic="VERTEX",
                source=f"#{identity}-vertices",
                offset="0",
            )
            add(
                triangles,
                "input",
                semantic="NORMAL",
                source=f"#{identity}-normals",
                offset="1",
            )
            add(triangles, "p", " ".join(f"{i} {i}" for face in faces for i in face))
            instance = add(
                add(scene, "node", id=identity + "-node"),
                "instance_geometry",
                url="#" + identity,
            )
            add(
                add(add(instance, "bind_material"), "technique_common"),
                "instance_material",
                symbol=material,
                target="#" + material,
            )
        # Material names may coincide with generated geometry/node IDs.
        used_ids = {element.get("id") for element in root.iter() if element.get("id")}

        def allocate_id(name):
            candidate = name
            while candidate in used_ids:
                candidate += "_"
            used_ids.add(candidate)
            return candidate

        material_ids = {}
        for name, properties in materials.items():
            material_id = allocate_id(name)
            effect_id = allocate_id(f"{name}-fx")
            material_ids[name] = material_id
            effect = add(effects, "effect", id=effect_id)
            technique = add(add(effect, "profile_COMMON"), "technique", sid="common")
            phong = add(technique, "phong")
            add(
                add(phong, "diffuse"),
                "color",
                " ".join(map(str, properties["diffuse"])),
            )
            add(
                add(phong, "specular"),
                "color",
                " ".join(map(str, properties.get("specular", [0.16, 0.16, 0.16, 1]))),
            )
            add(add(phong, "shininess"), "float", str(properties.get("shininess", 36)))
            if "emission" in properties:
                add(
                    add(phong, "emission"),
                    "color",
                    " ".join(map(str, properties["emission"])),
                )
            add(
                add(material_library, "material", id=material_id, name=name),
                "instance_effect",
                url=f"#{effect_id}",
            )
        for binding in root.iter(f"{{{NS}}}instance_material"):
            binding.set("target", "#" + material_ids[binding.get("symbol")])
        add(add(root, "scene"), "instance_visual_scene", url="#Scene")
        ET.indent(root)
        path.write_bytes(
            ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
        )


def write_revision(
    output, mesh_names, macro="visual_revision", label="Visual meshes SHA256"
):
    """Include the generated asset digest in expanded URDF provenance."""
    digest = hashlib.sha256()
    for name in mesh_names:
        digest.update((output / f"{name}.dae").read_bytes())
    revision = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    body = ET.SubElement(revision, "xacro:macro", {"name": macro})
    body.append(ET.Comment(f" {label}: {digest.hexdigest()} "))
    # Preserve the canonical formatting used by existing Xacro consumers.
    ET.indent(revision)
    (output / "revision.xacro").write_bytes(
        b'<?xml version="1.0"?>\n' + ET.tostring(revision, encoding="utf-8") + b"\n"
    )


def generate(source, output=None):
    """Build named visual meshes from a JSON recipe; no kinematic model is read."""
    source = Path(source)
    recipe = json.loads(source.read_text())
    output = Path(output) if output is not None else source.parent / "visual"
    materials = recipe["materials"]
    meshes = {}
    for name, shapes in recipe["links"].items():
        if not name or Path(name).name != name or name in (".", ".."):
            raise ValueError("mesh name must be a filename, not a path")
        shell = Shell(recipe.get("segments", 64))
        operations = {
            "body": shell.body,
            "lathe": shell.lathe,
            "ring": shell.ring,
            "bolts": shell.bolts,
        }
        for shape in shapes:
            parameters = dict(shape)
            operation = parameters.pop("shape")
            if operation not in operations:
                raise ValueError(f"unknown visual shape: {operation}")
            if parameters["material"] not in materials:
                raise ValueError(f"unknown material: {parameters['material']}")
            operations[operation](**parameters)
        meshes[name] = shell
    output.mkdir(parents=True, exist_ok=True)
    for name, shell in meshes.items():
        shell.save(output / f"{name}.dae", materials, recipe.get("metadata"))
    write_revision(output, meshes, **recipe.get("revision", {}))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recipe", type=Path, help="visual geometry/material JSON")
    parser.add_argument("--output", type=Path, help="default: visual/ next to recipe")
    args = parser.parse_args()
    print(generate(args.recipe, args.output))


if __name__ == "__main__":
    main()
