"""Read-only rosbag metadata and manifest helpers."""

import hashlib
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_manifest(source_bag, selected_count, source_hash=None):
    if selected_count <= 0:
        raise ValueError("selected frame count must be positive")
    return {
        "schema_version": 1,
        "selection": {"exact_rgbd_pairs": selected_count},
        "source_capture_sha256": source_hash or sha256(source_bag),
        "source_mutated": False,
    }
