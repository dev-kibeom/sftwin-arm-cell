#!/usr/bin/env python3
"""Generate the self-contained MoveIt static-scene artifact."""

import argparse
import hashlib
import json
from pathlib import Path


class StaticSceneValidationError(Exception):
    """Raised when canonical static-scene inputs violate their schema."""


def read_json(path):
    return json.loads(path.read_text())


def canonical_bytes(data):
    return json.dumps(data, indent=2, sort_keys=True) + "\n"


def validate(manifest, selection):
    if manifest.get("schema_version") != 1 or manifest.get("coverage") != "partial":
        raise StaticSceneValidationError(
            "unsupported canonical static-environment manifest"
        )
    if manifest.get("frame_id") != "world":
        raise StaticSceneValidationError(
            "static scene must be expressed in the world frame"
        )
    objects = manifest.get("objects")
    if not isinstance(objects, list) or len({o.get("id") for o in objects}) != len(
        objects
    ):
        raise StaticSceneValidationError("manifest objects must have unique ids")
    by_id = {o["id"]: o for o in objects}
    ids = selection.get("selected_ids")
    if selection.get("schema_version") != 1 or not isinstance(ids, list):
        raise StaticSceneValidationError("invalid MoveIt selection")
    if len(ids) != len(set(ids)) or any(object_id not in by_id for object_id in ids):
        raise StaticSceneValidationError(
            "selection contains unknown or duplicate object ids"
        )
    for obj in objects:
        if obj.get("geometry", {}).get("type") != "box":
            raise StaticSceneValidationError(
                f"unsupported geometry for {obj.get('id')}"
            )
        if len(obj["geometry"].get("dimensions_m", [])) != 3:
            raise StaticSceneValidationError(f"invalid dimensions for {obj['id']}")
        if (
            len(obj["pose"].get("position_m", [])) != 3
            or len(obj["pose"].get("quaternion_xyzw", [])) != 4
        ):
            raise StaticSceneValidationError(f"invalid pose for {obj['id']}")
    return by_id


def generate(manifest_path, selection_path, output_path):
    manifest = read_json(manifest_path)
    selection = read_json(selection_path)
    by_id = validate(manifest, selection)
    artifact = {
        "schema_version": 1,
        "source": {
            "canonical_manifest_sha256": hashlib.sha256(
                canonical_bytes(manifest).encode()
            ).hexdigest(),
            "selection_sha256": hashlib.sha256(
                canonical_bytes(selection).encode()
            ).hexdigest(),
        },
        "frame_id": manifest["frame_id"],
        "objects": [by_id[object_id] for object_id in selection["selected_ids"]],
    }
    output_path.write_text(canonical_bytes(artifact))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("selection", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.manifest, args.selection, args.output)


if __name__ == "__main__":
    main()
