"""Consume Bringup's Cell scene selection without owning compatibility policy.

The selected callable accepts ``configuration`` plus the existing Isaac scene
services and returns the Cell's scene context. Selection grants no support for
other runtime bindings (Action Graph, articulation or gripper).
"""
from dataclasses import dataclass
from importlib import import_module
import math

from scene_builder.model_build_configuration import ModelBuildConfiguration


def run_scene_build(*, configuration, **services):
    module, name = configuration['cell']['isaac']['scene_builder'].split(':')
    return getattr(import_module(module), name)(configuration=configuration, **services)


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
