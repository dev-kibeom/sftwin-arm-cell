"""Build/check the selected Assembly without starting Timeline or motion.

Run with Isaac Sim's python.sh and an exported ARM_CELL_RUNTIME_CONFIG.
--build uses the existing common Pre Build role; no alternate importer exists.
"""
import argparse
import math
import os
from pathlib import Path
import runpy
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--build', action='store_true', help='Run common Pre Build before checking')
    args = parser.parse_args()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from pxr import Usd, UsdPhysics
        from isaac_runtime.runtime_configuration import load_runtime_configuration
        from isaac_runtime.model_build_configuration import ModelBuildConfiguration
        from isaac_runtime.model_provenance import verify_current_artifact
        from isaac_runtime.robot_binding import (
            RobotBinding, author_home_targets, apply_joint_drive_profile,
        )
        root = Path(os.environ['SFTWIN_PROJECT_ROOT']).resolve()
        if args.build:
            runpy.run_path(
                str(root / 'infra/isaac_sim/scripts/0_pre_build.py'), run_name='__main__')
        config = load_runtime_configuration()
        model = ModelBuildConfiguration.from_runtime(config, root)
        verify_current_artifact(model.source, model.generated_urdf, model.usd, model.provenance,
                                xacro_arguments=model.xacro_arguments)
        stage = Usd.Stage.CreateInMemory()
        binding = RobotBinding.from_runtime(config)
        stage.DefinePrim(binding.robot_path, 'Xform').GetReferences().AddReference(str(model.usd))
        articulation = stage.GetPrimAtPath(binding.articulation_root_path)
        if not articulation.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError('Selected articulation root has no ArticulationRootAPI')
        if not stage.GetPrimAtPath(binding.base_link_path).IsValid():
            raise RuntimeError('Selected articulation base is absent')
        # These operations author an in-memory stage; no Timeline/physics is run.
        author_home_targets(stage, binding, UsdPhysics)
        apply_joint_drive_profile(stage, binding, UsdPhysics)
        prims = list(stage.Traverse())
        for name, position in zip(binding.joint_names, binding.home_positions):
            matches = [prim for prim in prims if prim.GetName() == name]
            if len(matches) != 1 or not matches[0].IsA(UsdPhysics.RevoluteJoint):
                raise RuntimeError(f'Selected arm joint is absent/ambiguous/not revolute: {name}')
            drive = UsdPhysics.DriveAPI.Get(matches[0], 'angular')
            target = drive.GetTargetPositionAttr().Get()
            if not math.isclose(target, math.degrees(position), abs_tol=1e-4):
                raise RuntimeError(f'HOME target differs from selected Robot: {name}')
            if drive.GetStiffnessAttr().Get() <= 0:
                raise RuntimeError(f'Joint drive has no positive stiffness: {name}')
            print(f'{name}: drive={drive.GetTypeAttr().Get()}, '
                  f'maxForce={drive.GetMaxForceAttr().Get()}', flush=True)
        for name in ('tool_mount', 'sf_grasp_tcp', *model.revolute_joints):
            matches = [prim for prim in prims if prim.GetName() == name]
            if len(matches) != 1:
                raise RuntimeError(f'Tool binding is absent/ambiguous: {name}')
            if any('physxMimicJoint' in attr.GetName() for attr in matches[0].GetAttributes()):
                raise RuntimeError(f'Unexpected PhysX mimic on {name}')
        print(f'STATIC MODEL PASS: {model.model_id}; Timeline was not started', flush=True)
    finally:
        sys.stdout.flush()
        app.close()


if __name__ == '__main__':
    main()
