"""Locate the repository root from relocatable M0609 verification files."""

from pathlib import Path


def repository_root(start):
    for candidate in (Path(start).resolve(), *Path(start).resolve().parents):
        if (candidate / "infra/isaac_sim/scripts/m0609_cell").is_dir() and (
            candidate / "ros2_ws/src/arm_cell"
        ).is_dir():
            return candidate
    raise RuntimeError(f"cannot locate repository root from {start}")


def local_artifact_bag(root, bag_name):
    """Return an optional private capture without coupling tests to a checkout path."""
    return Path(root) / ".local_artifacts" / "ros2_bags" / bag_name
