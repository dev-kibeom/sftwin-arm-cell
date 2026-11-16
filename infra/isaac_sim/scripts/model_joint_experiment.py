"""Isolated position-drive experiment: no Cell, ROS, grasp or payload runtime."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import runpy
import sys
import xml.etree.ElementTree as ET


def aligned_fixed_translation(urdf, base, tip):
    """C10's canonical TCP stays aligned with its housing +Z insertion axis."""
    by_child = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
    result, visited = [0., 0., 0.], set()
    while tip != base:
        joint = by_child.get(tip)
        if tip in visited or joint is None or joint.get('type') != 'fixed':
            raise ValueError('Tool TCP must be fixed to its housing')
        visited.add(tip)
        origin = joint.find('origin')
        if origin is not None:
            rotation = list(map(float, origin.get('rpy', '0 0 0').split()))
            offset = list(map(float, origin.get('xyz', '0 0 0').split()))
            if rotation != [0., 0., 0.] or len(offset) != 3 or not all(map(math.isfinite, offset)):
                raise ValueError('C10 TCP requires aligned finite translation')
            result = [a+b for a, b in zip(result, offset)]
        tip = joint.find('parent').get('link')
    return result


class ExperimentAborted(Exception):
    """Window closed before the requested measurement finished."""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--configuration', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--task-execution-library', help='Test bridge library for common native Task BT integration')
    parser.add_argument('--internal-grasp', action='store_true', help='Isolated logical bore grasp with teaching poses')
    parser.add_argument('--cycles', type=int, default=0, help='0 repeats until the window closes')
    args = parser.parse_args()
    if args.cycles < 0 or (args.headless and args.cycles == 0):
        parser.error('Headless execution requires --cycles N with N > 0')
    config = json.loads(Path(args.configuration).read_text())
    if config.get('execution_scope') != 'model_only':
        parser.error('A model-only configuration is required')
    if args.task_execution_library and not args.internal_grasp:
        parser.error('--task-execution-library requires --internal-grasp')
    if args.internal_grasp and 'internal_grasp_experiment' not in config:
        parser.error('Selected profile has no internal grasp teaching/fixture configuration')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless})
    report = {'status': 'ABORTED', 'revision': os.environ.get('ARM_CELL_EXPERIMENT_REVISION'),
              'configuration_sha256': hashlib.sha256(Path(args.configuration).read_bytes()).hexdigest(),
              'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'scope': 'model and joint actuation only; no grasp validation', 'cycles': []}
    try:
        import numpy as np
        import omni.usd
        from pxr import Usd, UsdGeom, UsdLux, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaac_runtime.model_build_configuration import ModelBuildConfiguration
        from isaac_runtime.robot_binding import RobotBinding, author_home_targets, apply_joint_drive_profile

        root = Path(os.environ['SFTWIN_PROJECT_ROOT'])
        model = ModelBuildConfiguration.from_runtime(config, root)
        # Use the production importer and its existing parse_mimic=False policy.
        # All dependent SI targets below come from the URDF-owned mimic data.
        runpy.run_path(str(root / 'infra/isaac_sim/scripts/isaac_runtime/runtime_steps/build_robot_usd.py'),
                       init_globals={'runtime_configuration': config})
        urdf = ET.parse(model.generated_urdf).getroot()
        report['generated_urdf_sha256'] = hashlib.sha256(model.generated_urdf.read_bytes()).hexdigest()
        from internal_grasp_geometry import ToolGeometry
        tool_geometry = ToolGeometry(urdf, config['tool']['runtime'])
        expected_tcp = aligned_fixed_translation(urdf, tool_geometry.base_link, 'sf_grasp_tcp')
        expected_collisions = {link.get('name'): len(link.findall('collision'))
                               for link in urdf.findall('link')
                               if link.get('name').startswith('gripper_') and link.findall('collision')}
        world = World(stage_units_in_meters=1., physics_dt=1/120, rendering_dt=1/60)
        stage = omni.usd.get_context().get_stage()
        robot_path = '/World/Robot'
        stage.DefinePrim(robot_path, 'Xform').GetReferences().AddReference(str(model.usd))
        assembly = config['model_configuration']['isaac']
        articulation_path = robot_path + '/' + assembly['articulation_root']
        if not stage.GetPrimAtPath(articulation_path).HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError('Imported model has no ArticulationRootAPI')
        robot_settings = config['robot'].get('isaac', {})
        binding = RobotBinding(robot_path, articulation_path, robot_path + '/base_link',
                               tuple(config['robot']['motion']['arm_joint_names']),
                               tuple(config['home_joint_positions']),
                               robot_settings.get('joint_drive_stiffness_scales', {}),
                               robot_settings.get('joint_drive_max_forces', {}),
                               robot_settings.get('joint_drive_damping', {}))
        author_home_targets(stage, binding, UsdPhysics)
        apply_joint_drive_profile(stage, binding, UsdPhysics)
        if args.internal_grasp:
            # Fixture alignment needs sub-mm tracking. Tune only this isolated
            # experiment; the selected model and operational drive policy stay intact.
            scale = config['internal_grasp_experiment']['arm_drive_stiffness_scale']
            for prim in stage.Traverse():
                if prim.GetName() in binding.joint_names and prim.HasAPI(UsdPhysics.DriveAPI, 'angular'):
                    drive = UsdPhysics.DriveAPI(prim, 'angular')
                    drive.GetStiffnessAttr().Set(drive.GetStiffnessAttr().Get()*scale)
                    drive.GetDampingAttr().Set(drive.GetDampingAttr().Get()*math.sqrt(scale))
        settings = config['tool']['runtime']
        linkage = settings['joint_model']
        names = list(linkage)
        tip_links = settings.get('finger_tip_links', [
            urdf.find(f"joint[@name='{name}']/child").get('link') for name in names])
        prims = {prim.GetName(): prim for prim in stage.Traverse() if str(prim.GetPath()).startswith(robot_path + '/')}
        drives = {}
        for name, joint in linkage.items():
            prim = prims[name]
            if not prim.IsA(UsdPhysics.PrismaticJoint):
                raise RuntimeError(f'Prismatic joint missing: {name}')
            schema = UsdPhysics.PrismaticJoint(prim)
            if not (math.isclose(schema.GetLowerLimitAttr().Get(), joint['lower'], abs_tol=1e-7)
                    and math.isclose(schema.GetUpperLimitAttr().Get(), joint['upper'], abs_tol=1e-7)):
                raise RuntimeError(f'Imported limits differ from URDF: {name}')
            drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(settings['drive_stiffness'])
            drive.CreateDampingAttr(settings['drive_damping'])
            drive.CreateMaxForceAttr(joint['effort'])
            drive.CreateTargetPositionAttr(joint['lower'])
            drives[name] = drive
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
        if not args.headless:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=[1.6, 1.6, 1.3], target=[0, 0, .6])
        robot = world.scene.add(SingleArticulation(prim_path=robot_path, name='model_experiment'))
        world.reset()
        arm_indices = [robot.get_dof_index(name) for name in binding.joint_names]
        robot.set_joint_positions(np.array(binding.home_positions), joint_indices=np.array(arm_indices))
        indices = [robot.get_dof_index(name) for name in names]
        if len(set(indices)) != len(names):
            raise RuntimeError('Tool DOF binding is ambiguous')
        tolerance = settings['position_tolerance_m']
        primary = linkage[settings['primary_joint']]
        lower, upper = primary['lower'], primary['upper']
        observed_min = np.full(len(names), np.inf)
        observed_max = np.full(len(names), -np.inf)
        max_velocity = np.zeros(len(names))
        sample_count = 0
        max_sync = 0.

        def command(position):
            for name, joint in linkage.items():
                mimic = joint.get('mimic', {})
                value = position * mimic.get('multiplier', 1.) + mimic.get('offset', 0.)
                if not joint['lower'] <= value <= joint['upper']:
                    raise RuntimeError(f'Target outside URDF limits: {name}')
                drives[name].GetTargetPositionAttr().Set(value)

        def step():
            nonlocal max_sync, sample_count
            if not app.is_running():
                raise ExperimentAborted()
            world.step(render=not args.headless)
            if not app.is_running():
                raise ExperimentAborted()
            values = np.array(robot.get_joint_positions(joint_indices=np.array(indices)))
            if not np.all(np.isfinite(values)):
                raise RuntimeError('Nonfinite measured joint positions')
            for value, name in zip(values, names):
                if not linkage[name]['lower'] - tolerance <= value <= linkage[name]['upper'] + tolerance:
                    raise RuntimeError(f'Measured joint limit exceeded: {name}={value}')
            # C10's direct 1:1 mimic: all three measured translations must agree.
            max_sync = max(max_sync, float(np.ptp(values)))
            if max_sync > tolerance:
                raise RuntimeError(f'Measured finger synchronization error: {max_sync} m')
            velocities = np.array(robot.get_joint_velocities(joint_indices=np.array(indices)))
            if not np.all(np.isfinite(velocities)):
                raise RuntimeError('Nonfinite measured joint velocities')
            for value, name in zip(velocities, names):
                if abs(value) > linkage[name]['velocity'] + 1e-5:
                    raise RuntimeError(f'Measured velocity limit exceeded: {name}={value}')
            np.minimum(observed_min, values, out=observed_min)
            np.maximum(observed_max, values, out=observed_max)
            np.maximum(max_velocity, np.abs(velocities), out=max_velocity)
            sample_count += 1
            return values

        def settle(position):
            command(position)
            for _ in range(120):
                values = step()
            if np.max(np.abs(values - position)) > tolerance:
                raise RuntimeError(f'Position drive failed to reach {position}: {values}')
            return values.tolist()

        def geometry():
            cache = UsdGeom.XformCache()
            base = cache.GetLocalToWorldTransform(prims[tool_geometry.base_link]).GetInverse()
            positions = []
            for name in tip_links:
                transform = cache.GetLocalToWorldTransform(prims[name]) * base
                positions.append(list(transform.ExtractTranslation()))
            tcp = cache.GetLocalToWorldTransform(prims['sf_grasp_tcp']) * base
            if not np.allclose(tcp.ExtractTranslation(), expected_tcp, atol=1e-5):
                raise RuntimeError('Imported TCP differs from selected Tool URDF')
            return np.array(positions)

        collision_counts = {}
        for name, expected_count in expected_collisions.items():
            count = sum(prim.HasAPI(UsdPhysics.CollisionAPI)
                        for prim in Usd.PrimRange(prims[name], Usd.TraverseInstanceProxies()))
            if count != expected_count:
                raise RuntimeError(f'Collision decomposition lost on {name}')
            collision_counts[name] = count
        if args.internal_grasp:
            from internal_grasp_experiment import run_internal_grasp
            report['scope'] = 'isolated teaching-pose logical internal grasp; no Mission or physical grasp'
            report['execution_mode'] = 'common_task_bt_fixture' if args.task_execution_library else 'isolated_fixture'
            run_internal_grasp(config, tool_geometry, stage, world, robot, prims, arm_indices,
                               indices, step, command, report, args.cycles, args.task_execution_library)
            report.update(status='PASS', model_id=model.model_id,
                          max_synchronization_error_m=max_sync, sample_count=sample_count,
                          tcp_translation_m=expected_tcp, observed_tip_links=tip_links,
                          collision_counts=collision_counts,
                          usd_sha256=hashlib.sha256(model.usd.read_bytes()).hexdigest())
            print('C10 INTERNAL GRASP EXPERIMENT PASS', flush=True)
            return
        cycle = 0
        while app.is_running() and (args.cycles == 0 or cycle < args.cycles):
            opened = settle(lower)
            open_geometry = geometry()
            endpoints = []
            for start, finish in ((lower, upper), (upper, lower)):
                frames = math.ceil(settings['sweep_duration_s'] * 120)
                if abs(finish - start) / settings['sweep_duration_s'] > min(j['velocity'] for j in linkage.values()):
                    raise RuntimeError('Sweep exceeds URDF velocity limit')
                for frame in range(1, frames + 1):
                    if not app.is_running():
                        return
                    command(start + (finish - start) * frame / frames)
                    step()
                endpoints.append(settle(finish))
                if finish == upper:
                    displacement = geometry() - open_geometry
                    expected = np.array([[-math.cos(i * 2*math.pi/3), -math.sin(i * 2*math.pi/3), 0]
                                         for i in range(3)]) * (upper - lower)
                    if not np.allclose(displacement, expected, atol=tolerance):
                        raise RuntimeError(f'Finger displacement is not radial contraction: {displacement}')
            report['cycles'].append({'open_m': opened, 'contracted_m': endpoints[0],
                                     'expanded_m': endpoints[1], 'contraction_vectors_m': displacement.tolist()})
            cycle += 1
            print(f'C10 cycle {cycle}: measured radial contraction/expansion PASS', flush=True)
            if args.cycles == 0:
                # Keep a bounded report for long interactive experiments.
                report['cycles'] = report['cycles'][-10:]
        if not cycle or (args.cycles and cycle != args.cycles):
            return
        report.update(status='PASS', model_id=model.model_id, articulation_dof_names=robot.dof_names,
                      max_synchronization_error_m=max_sync, collision_counts=collision_counts,
                      tcp_translation_m=expected_tcp, observed_tip_links=tip_links,
                      observed_min_m=observed_min.tolist(), observed_max_m=observed_max.tolist(),
                      max_abs_velocity_m_s=max_velocity.tolist(), sample_count=sample_count,
                      usd_sha256=hashlib.sha256(model.usd.read_bytes()).hexdigest())
        print('MODEL JOINT EXPERIMENT PASS', flush=True)
    except ExperimentAborted:
        print('Model experiment aborted before completion', flush=True)
    except Exception as exc:
        report.update(status='FAIL', error=str(exc))
        raise
    finally:
        Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
        print(f'Experiment report: {args.report}', flush=True)
        sys.stdout.flush()
        app.close()


if __name__ == '__main__':
    main()
