"""Isaac radial actuator binding; primary encoder feedback and physical drives."""
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

IDENTITY_KEYS = ('material_id', 'supply_id', 'slot_id', 'supply_epoch', 'reservation_id')


def contact_links_by_joint(urdf, joint_names, tip_links):
    """Resolve selected contact tips through fixed links, in measured DOF order."""
    parents={joint.find('child').get('link'):joint for joint in urdf.findall('joint')}
    if len(set(tip_links))!=len(joint_names):
        raise ValueError('One distinct contact tip per radial joint is required')
    result=[]
    for name in joint_names:
        joint=urdf.find(f"joint[@name='{name}']")
        if joint is None:raise ValueError(f'Unknown radial joint: {name}')
        child=joint.find('child').get('link')
        matches=[]
        for tip in tip_links:
            cursor=tip
            while cursor!=child:
                parent=parents.get(cursor)
                if parent is None or parent.get('type')!='fixed':break
                cursor=parent.find('parent').get('link')
            if cursor==child:matches.append(tip)
        if len(matches)!=1:
            raise ValueError(f'Contact tip must belong to radial joint: {name}')
        result.append(matches[0])
    return result


def reserved_identity_matches(read_attribute, reference):
    return (read_attribute('arm_cell:occupied') is True and
            all(read_attribute('arm_cell:' + key) == reference[key] for key in IDENTITY_KEYS))


def finger_contact_forces(snapshot, fingers, dt, *, target=None):
    if snapshot is None or not math.isfinite(dt) or dt <= 0:
        return None
    forces = [0.] * len(fingers)
    for contact in snapshot:
        if contact['lost']:
            continue
        actors = contact['actor0'], contact['actor1']
        if target is not None and target not in actors:
            continue
        touched = [finger for finger in fingers if finger in actors and finger != target]
        if not touched:
            continue
        force = sum(math.sqrt(sum(float(v)**2 for v in point['impulse'])) / dt
                    for point in contact['points'])
        if not math.isfinite(force):
            return None
        for finger in touched:
            forces[fingers.index(finger)] += force
    return forces


def target_contact_forces(snapshot, target, fingers, dt):
    if not target:
        return None
    return finger_contact_forces(snapshot, fingers, dt, target=target)


def prepare_drives(stage, robot_binding, settings):
    from pxr import UsdPhysics, PhysxSchema
    from isaac_runtime.robot_binding import _robot_joints
    joints = _robot_joints(stage, robot_binding, settings['joint_model'])
    for name, model in settings['joint_model'].items():
        prim = joints[name]
        if not prim.IsA(UsdPhysics.PrismaticJoint):
            raise RuntimeError(f'Radial Tool joint must be prismatic: {name}')
        joint = UsdPhysics.PrismaticJoint(prim)
        if not (math.isclose(joint.GetLowerLimitAttr().Get(), model['lower'], abs_tol=1e-7)
                and math.isclose(joint.GetUpperLimitAttr().Get(), model['upper'], abs_tol=1e-7)):
            raise RuntimeError(f'Radial Tool joint limits differ from selected model: {name}')
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(
            model['velocity'] * settings.get('max_velocity_fraction', 1.))
        drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(settings['drive_stiffness'])
        drive.CreateDampingAttr(settings['drive_damping'])
        drive.CreateMaxForceAttr(model['effort'])
        drive.CreateTargetPositionAttr(model['upper'])


class IsaacRadialBinding:
    def __init__(self, configuration, robot_binding):
        import omni.usd
        from pxr import UsdGeom, UsdPhysics, PhysxSchema
        from omni.isaac.dynamic_control import _dynamic_control as api
        from isaac_runtime.model_build_configuration import ModelBuildConfiguration
        from gripper_runtime.radial_geometry import ToolGeometry
        self.stage = omni.usd.get_context().get_stage()
        self.robot_binding = robot_binding
        if not math.isclose(UsdGeom.GetStageMetersPerUnit(self.stage), 1., abs_tol=1e-9):
            raise RuntimeError('Radial Tool requires a metre-denominated Stage')
        self.settings = configuration['tool']['runtime']
        self.physical_settings = self.settings['physical']
        project_root = Path(os.environ.get('SFTWIN_PROJECT_ROOT', Path(__file__).resolve().parents[4]))
        model = ModelBuildConfiguration.from_runtime(configuration, project_root)
        self.geometry = ToolGeometry(ET.parse(model.generated_urdf).getroot(), self.settings)
        # Resolve configured requirements before subscribing the real-time Tool session.
        import json
        for cycle in json.loads(configuration.get('motion', {}).get('configured_grasp_cycles_json', '{}')).values():
            grasp=cycle['grasp']
            self.geometry.solve_bore(grasp['insertion_depth_m'],grasp['thickness_m'],grasp['inner_radius_m'])
        self.dc, self.api = api.acquire_dynamic_control_interface(), api
        if not self.dc.is_simulating():
            raise RuntimeError('Radial Tool requires Play and a Physics frame')
        self.articulation = self.dc.get_articulation(robot_binding.base_link_path)
        if self.articulation == api.INVALID_HANDLE:
            raise RuntimeError('Selected radial Articulation is unavailable')
        self.models = self.settings['joint_model']
        self.dofs = {name: self.dc.find_articulation_dof(self.articulation, name) for name in self.models}
        if (len(self.dofs) != 3 or len(set(self.dofs.values())) != 3
                or any(dof == api.INVALID_HANDLE for dof in self.dofs.values())):
            raise RuntimeError('Three radial actuator DOFs required')
        self.primary = self.settings['primary_joint']
        # Preserve selected model order with the primary first for actuator feedback.
        self.names = [self.primary] + [name for name in self.models if name != self.primary]
        for name, dof in self.dofs.items():
            props = self.dc.get_dof_properties(dof)
            props.drive_mode = api.DRIVE_FORCE
            props.stiffness = self.settings['drive_stiffness']
            props.damping = self.settings['drive_damping']
            props.max_effort = self.models[name]['effort']
            props.max_velocity = self.models[name]['velocity'] * self.settings.get('max_velocity_fraction', 1.)
            if not self.dc.set_dof_properties(dof, props):
                raise RuntimeError(f'Failed to configure radial drive: {name}')
        prefix = robot_binding.robot_path.rstrip('/') + '/'
        selected = {}
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).startswith(prefix):
                selected.setdefault(prim.GetName(), []).append(prim)
        def unique(name):
            matches = selected.get(name, [])
            if len(matches) != 1:
                raise RuntimeError(f'Radial Tool prim is missing or ambiguous: {name}')
            return matches[0]
        self.tcp_path = str(unique('sf_grasp_tcp').GetPath())
        fingers = []
        # Match finger order to measured joint order, independently of dict sorting.
        urdf = self.geometry.urdf
        for link in contact_links_by_joint(urdf,self.names,self.settings['finger_tip_links']):
            prim = unique(link)
            while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim = prim.GetParent()
            if not prim:
                raise RuntimeError(f'Radial finger has no independent rigid body: {name}')
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            fingers.append(str(prim.GetPath()))
        if len(set(fingers)) != 3:
            raise RuntimeError('Radial fingers require independent physical bodies')
        self.finger_bodies = tuple(fingers)
        if not self.reconcile_stale_owned_joint():
            raise RuntimeError('Cannot clear stale attachment before radial Physical mode')

    def measure_primary(self):
        state = self.dc.get_dof_state(self.dofs[self.primary], self.api.STATE_ALL)
        position = float(state.pos)
        model = self.models[self.primary]
        tolerance = self.settings['position_tolerance_m']
        if not math.isfinite(position) or not model['lower']-tolerance <= position <= model['upper']+tolerance:
            raise RuntimeError('Invalid measured radial primary position')
        return position

    def write_primary(self, position):
        targets = {}
        for name, model in self.models.items():
            mimic = model.get('mimic', {})
            value = position*mimic.get('multiplier', 1.) + mimic.get('offset', 0.)
            if not math.isfinite(value) or not model['lower'] <= value <= model['upper']:
                raise ValueError(f'Radial target outside selected model limits: {name}')
            targets[name] = value
        for name, value in targets.items():
            if not self.dc.set_dof_position_target(self.dofs[name], value):
                raise RuntimeError(f'Failed to write radial drive target: {name}')
        self.dc.wake_up_articulation(self.articulation)

    def reconcile_stale_owned_joint(self):
        from gripper_runtime.grasp_joint import UsdFixedJointOwner
        return UsdFixedJointOwner(self.stage, self.tcp_path).reconcile_stale()

    def reset_stopped_tool(self, actuator):
        import omni.timeline
        if not omni.timeline.get_timeline_interface().is_stopped():
            return False
        # Articulation handles can already be invalid at the STOP callback.
        # Reset only authored drive targets; next Play creates fresh bindings.
        prepare_drives(self.stage, self.robot_binding, self.settings)
        actuator.reset_stopped_state()
        return True

    def clear(self):
        pass

    def shutdown(self):
        pass
