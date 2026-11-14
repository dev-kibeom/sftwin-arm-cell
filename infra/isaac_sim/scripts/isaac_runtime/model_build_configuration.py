"""Resolve selected Assembly inputs and derived Isaac artifact names."""
from dataclasses import dataclass
from pathlib import Path
import os
import re


@dataclass(frozen=True)
class ModelBuildConfiguration:
    model_id: str
    source: Path
    xacro_arguments: tuple
    generated_urdf: Path
    usd: Path
    revolute_joints: tuple

    @classmethod
    def from_runtime(cls, configuration, project_root):
        model = configuration['model_configuration']
        identity = model['model_id']
        # Identity is also a filename, never a path or an output-directory override.
        if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', identity):
            raise ValueError('Assembly model_id must be a filename-safe identity')
        binding = model['isaac']
        assets = Path(project_root) / 'infra/isaac_sim/assets'
        source = Path(binding.get('urdf', model['urdf']))
        # Keep Xacro's source-relative spelling of mesh paths. Canonical output
        # hashes (and the legacy Scene verifier) include that spelling.
        mesh_root = source.parent / os.path.relpath(model['mesh_root'], source.parent)
        return cls(
            identity,
            source,
            (f"mesh_root:={mesh_root}",),
            assets / 'urdf/generated' / f'{identity}.urdf',
            assets / 'usd' / f'{identity}_generated.usd',
            tuple(binding.get('revolute_joints', ())),
        )

    @property
    def provenance(self):
        return self.usd.with_suffix('.provenance.json')

    @property
    def base_layer(self):
        return self.usd.parent / 'configuration' / f'{self.usd.stem}_base.usd'
