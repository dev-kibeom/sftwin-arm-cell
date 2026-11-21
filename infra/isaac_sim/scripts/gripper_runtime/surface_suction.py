"""Isaac Sim 5.1 Surface Gripper with device-local holding observations."""
import time
from uuid import UUID

from gripper_runtime.configured_manager import ConfiguredGraspManager
from gripper_runtime.grasp_attachment import HoldingObservation, HoldingState
from gripper_runtime.tool_plugin import ToolIntent, ToolRuntime


class SurfaceCapture:
    requires_capture_reference = False

    @staticmethod
    def decode_reference(packet):
        if not isinstance(packet, dict) or packet.get('strategy') != 'surface_suction':
            return None
        execution = packet.get('motion_goal_uuid')
        try:
            UUID(execution)
        except (TypeError, ValueError, AttributeError):
            return None
        return dict(strategy='surface_suction', motion_goal_uuid=execution)

    @staticmethod
    def select(reference):
        return reference


class SuctionActuator:
    def __init__(self, device):
        self.device = device
        self.intent = None
        self.objects = ()
        self.status = None
        self.seen_held = self.holding_lost = self.released_held_object = False
        self._sample = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 0)

    def read(self):
        try:
            self.status, self.objects = self.device.sample()
        except (ValueError, RuntimeError):
            self.status, self.objects = None, ()

    def holding(self):
        # Model the cup's attachment feedback; object paths are diagnostic only.
        return self.status == 'Closed' and bool(self.objects)

    def command(self, intent):
        if intent is ToolIntent.GRASP:
            self.seen_held = self.holding_lost = self.released_held_object = False
            self.device.close()
        elif intent is ToolIntent.RELEASE:
            self.read()
            self.released_held_object = self.seen_held and not self.holding_lost and self.holding()
            self.device.open()
        else:
            self.cancel_motion()
            return
        self.intent = intent

    def cancel_motion(self):
        # Stop pending retries while preserving a closed device on interruption.
        if self.intent is ToolIntent.GRASP:
            self.read()
            if self.status == 'Closing':
                self.device.open()
        self.intent = None

    def update(self, dt):
        self.read()
        state = HoldingState.UNKNOWN
        if self.intent is ToolIntent.GRASP:
            if self.holding():
                state = HoldingState.HELD
                self.seen_held = True
            elif self.seen_held:
                self.holding_lost = True
        elif self.intent is ToolIntent.RELEASE and self.status == 'Open' and not self.objects:
            state = HoldingState.RELEASED
        self._sample = HoldingObservation(state, time.monotonic(), self._sample.sequence + 1)

    def holding_observation(self):
        return self._sample

    def observation(self):
        return dict(surface_status=self.status, gripped_objects=list(self.objects))


class SurfaceManager(ConfiguredGraspManager):
    diagnostic_prefix = 'suction'

    def _record(self, state):
        attached = bool(self.gripper.objects)
        if attached != self.attachment_owned() and state is self._holding.state:
            self._sequence += 1
        self._attached = attached
        super()._record(state)

    def attachment_owned(self):
        return getattr(self, '_attached', False)

    def release_receipt(self):
        if self.gripper.released_held_object:
            return super().release_receipt()
        return None

    def diagnostics(self):
        return self.gripper.observation()


def prepare(*, configuration, stage):
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.robot.surface_gripper')
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from usd.schema.isaac import robot_schema as schema

    root = stage.GetPrimAtPath(configuration['cell']['isaac']['robot_prim_path'])
    settings = configuration['tool']['runtime']

    def link(name):
        prim, = (p for p in Usd.PrimRange(root) if p.GetName() == name)
        return prim

    cup, tcp = link(settings['cup_link']), link('sf_grasp_tcp')
    if not cup.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Suction cup requires a rigid body')
    surface = schema.CreateSurfaceGripper(stage, str(root.GetPath())+'/SurfaceGripper')
    joint = UsdPhysics.Joint.Define(stage, surface.GetPath().AppendChild('Attachment'))
    joint.CreateBody0Rel().SetTargets([cup.GetPath()])
    cache = UsdGeom.XformCache()
    tcp_world = cache.GetLocalToWorldTransform(tcp)
    local = tcp_world * cache.GetLocalToWorldTransform(cup).GetInverse()
    # Give the idle D6 two unambiguous actors and coincident joint frames.
    # The extension retargets body1 to the raycast hit during GRASP.
    mount = link(settings['carrier_link'])
    joint.CreateBody1Rel().SetTargets([mount.GetPath()])
    mount_local = tcp_world * cache.GetLocalToWorldTransform(mount).GetInverse()
    joint.CreateLocalPos1Attr(Gf.Vec3f(mount_local.ExtractTranslation()))
    joint.CreateLocalRot1Attr(Gf.Quatf(mount_local.ExtractRotationQuat()))
    joint.CreateLocalPos0Attr(Gf.Vec3f(local.ExtractTranslation()))
    joint.CreateLocalRot0Attr(Gf.Quatf(local.ExtractRotationQuat()))
    joint.CreateJointEnabledAttr(True)
    joint.CreateExcludeFromArticulationAttr(True)
    for axis in ('transX', 'transY', 'rotX', 'rotY', 'rotZ'):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr(1.)
        limit.CreateHighAttr(-1.)
    # Allow axial compliance at the cup while locking lateral slip and rotation.
    axial = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), 'transZ')
    axial.CreateLowAttr(0.)
    axial.CreateHighAttr(settings['max_grip_distance_m'])
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'transZ')
    drive.CreateStiffnessAttr(5000.)
    drive.CreateDampingAttr(100.)
    schema.ApplyAttachmentPointAPI(joint.GetPrim())
    joint.GetPrim().GetAttribute(schema.Attributes.FORWARD_AXIS.name).Set('Z')
    joint.GetPrim().GetAttribute(schema.Attributes.CLEARANCE_OFFSET.name).Set(.0002)
    surface.GetRelationship(schema.Relations.ATTACHMENT_POINTS.name).SetTargets([joint.GetPath()])
    for attribute, value in (
            (schema.Attributes.MAX_GRIP_DISTANCE, settings['max_grip_distance_m']),
            (schema.Attributes.COAXIAL_FORCE_LIMIT, settings['coaxial_force_limit_n']),
            (schema.Attributes.SHEAR_FORCE_LIMIT, settings['shear_force_limit_n']),
            (schema.Attributes.RETRY_INTERVAL, settings['retry_interval_s'])):
        surface.GetAttribute(attribute.name).Set(value)


class IsaacSurfaceDevice:
    def __init__(self, robot_path):
        from isaacsim.robot.surface_gripper import _surface_gripper
        self.api = _surface_gripper.acquire_surface_gripper_interface()
        self.status_type = _surface_gripper.GripperStatus
        self.path = robot_path + '/SurfaceGripper'

    def close(self):
        if not self.api.close_gripper(self.path):
            raise ValueError('Surface Gripper rejected GRASP')

    def open(self):
        if not self.api.open_gripper(self.path) and self.sample() != ('Open', ()):
            raise ValueError('Surface Gripper rejected RELEASE')

    def sample(self):
        return (self.status_type(self.api.get_gripper_status(self.path)).name,
                tuple(self.api.get_gripped_objects(self.path)))

    def reset_stopped_tool(self, actuator):
        actuator.intent = None
        return True


def suction_runtime(profile, binding):
    capture = SurfaceCapture()
    device = IsaacSurfaceDevice(binding.robot_path)
    actuator = SuctionActuator(device)
    tool = ToolRuntime(actuator, frozenset(profile['runtime']['capabilities']),
                       lambda intent, duration: actuator.command(intent), capture)
    tool.grasp_factory = lambda **services: SurfaceManager(binding=device, **services)
    return tool
