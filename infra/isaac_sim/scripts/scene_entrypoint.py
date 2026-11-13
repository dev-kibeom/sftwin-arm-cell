"""Common Isaac Scene selection from an exported Runtime Configuration.

Import this module from the Isaac scripts root. Cell implementations are named
by fully qualified module:callable references relative to that root; selecting
one does not enable its Action Graph, articulation or gripper runtime bindings.
The callable accepts configuration and the existing Isaac scene services.
"""
from importlib import import_module


def run_scene_build(*, configuration, **services):
    module, name = configuration['cell']['isaac']['scene_builder'].split(':')
    return getattr(import_module(module), name)(configuration=configuration, **services)
