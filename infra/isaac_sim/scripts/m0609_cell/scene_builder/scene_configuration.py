"""Inputs consumed by the M0609 scene implementation."""
from dataclasses import dataclass
import math

from .model_build_configuration import ModelBuildConfiguration


@dataclass(frozen=True)
class SceneConfiguration:
    model: ModelBuildConfiguration
    robot_prim_path: str
    position_m: tuple
    rotation_deg: tuple
    geometry: str

    @classmethod
    def from_runtime(cls, configuration, project_root):
        cell = configuration['cell']
        transform = cell['world_to_base']
        return cls(
            ModelBuildConfiguration.from_runtime(configuration, project_root),
            cell['isaac']['robot_prim_path'],
            tuple(transform[key] for key in ('x', 'y', 'z')),
            tuple(math.degrees(transform[key]) for key in ('roll', 'pitch', 'yaw')),
            cell['geometry'],
        )
