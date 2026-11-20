"""Cell-owned pendant drafts, persistence and asynchronous Motion requests."""

from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np

from .teaching_program import (
    JOINT_MOVEMENTS, motion_name,
    apply_layout, author_equipment_frames, base_matrix, base_pose_to_frame,
    capture_point, frame_pose_to_base, matrix_pose7, pose7_matrix, pose_matrix,
    matrix_pose, resolve_point, resolve_program, resolve_waypoint, validate_program,
)


class TeachingSession:
    def __init__(self, adapter, clock=time.monotonic):
        self.adapter, self.clock = adapter, clock
        self.configuration = adapter.configuration
        self.active = validate_program(self.configuration['cell']['teaching_program'])
        self.draft = deepcopy(self.active)
        self.path = Path(self.configuration['cell']['teaching_file'])
        self.saved = deepcopy(self.active) if self.path.is_file() else None
        self.manual_mode = False
        self.revision = 0
        self.detail = 'Select Manual Teaching to jog or test a point.'
        self.pending = None
        self.pose = self.joint_positions = None
        self.pose_at = self.state_at = float('-inf')
        self.motion_idle = self.motion_ready = False
        self.preview_path = '/World/TapeTeachingPreview'
        self._apply_snapshot = None
        self.apply_uncertain = False
        self.motion_uncertain = False
        self.requires_restart = False
        self.motion_instance = self.uncertain_instance = ''
        from isaac_runtime.cell_teaching import register
        register(self)
        names = self.configuration.get('robot', {}).get('motion', {}).get('arm_joint_names', [])
        self.adapter.sidecar.send(dict(op='teaching_settings', arm_joint_names=names))

    @property
    def world_to_base(self):
        return self.configuration['cell']['world_to_base']

    def _cell_busy(self):
        return bool(self.adapter._pending_cycle or self.adapter.supply.reservation_id)

    def status(self):
        now = self.clock()
        return dict(program=deepcopy(self.draft), active_program=deepcopy(self.active),
                    revision=self.revision, manual_mode=self.manual_mode,
                    pending=self.pending is not None, current_pose=deepcopy(self.pose),
                    current_pose6=matrix_pose(pose7_matrix(self.pose)) if self.pose is not None else None,
                    current_pose_valid=self.pose is not None and now-self.pose_at <= .5,
                    motion_ready=self.motion_ready and now-self.state_at <= .5,
                    dirty=self.saved != self.draft, unapplied=self.active != self.draft,
                    path=str(self.path), detail=self.detail)

    def consume(self, packet):
        if packet.get('op') == 'teaching_state':
            self.state_at = self.clock()
            self.motion_idle = packet.get('motion_idle') is True
            self.motion_ready = packet.get('motion_ready') is True
            self.motion_instance = str(packet.get('motion_instance', ''))
            if packet.get('pose_valid') is True:
                # Validate FK output before permitting capture or jog.
                try:
                    self.pose = matrix_pose7(pose7_matrix(packet['current_pose']))
                    self.joint_positions = packet.get('joint_positions')
                    self.pose_at = self.clock()
                except (ValueError, TypeError, KeyError):
                    self.pose_at = float('-inf')
            else:
                self.pose_at = float('-inf')
        if packet.get('op') != 'teaching_result' or self.pending is None:
            return
        if packet.get('request_id') != self.pending['id']:
            return
        operation = self.pending['operation']
        previous_instance = self.pending['instance']
        self.pending = None
        success = packet.get('success') is True
        self.detail = str(packet.get('detail') or ('Teaching request completed.' if success else 'Teaching request failed.'))
        if packet.get('uncertain'):
            if operation == 'apply':
                self.apply_uncertain = True
            else:
                self.motion_uncertain = True
                self.requires_restart = packet.get('requires_restart') is True
                self.uncertain_instance = previous_instance
        if success and operation == 'apply':
            candidate = self._apply_snapshot
            self.apply_uncertain = True
            try:
                if candidate['frames'] != self.active['frames']:
                    if not self.adapter.timeline.is_stopped() or self.adapter.conveyor.active_roll_paths:
                        raise ValueError('Timeline or material changed while applying equipment placement; stop and reapply.')
                apply_layout(self.configuration, candidate)
                author_equipment_frames(self.adapter.stage, candidate, self.configuration)
                motions = self._motions(candidate)
                self.configuration['motion']['taught_motions_json'] = json.dumps(motions, allow_nan=False)
                self.active = deepcopy(candidate)
                try:
                    import os
                    from isaac_runtime.live_test_session import record_configuration_snapshot
                    descriptor = os.environ.get('SFTWIN_LIVE_TEST_SESSION_PATH')
                    if descriptor:
                        record_configuration_snapshot(descriptor, self.configuration, 'isaac', 'teaching_applied')
                except Exception as error:
                    import logging
                    logging.getLogger(__name__).warning('Applied teaching evidence unavailable: %s', error)
                self.apply_uncertain = False
                self.motion_uncertain = self.requires_restart = False
                self.detail = 'Teaching applied. Saved and applied versions are tracked separately.'
            except (ValueError, RuntimeError, KeyError, TypeError) as error:
                self.detail = 'Motion accepted teaching but Scene application failed; remain in Manual Teaching and reapply: '+str(error)
        self._apply_snapshot = None
        self.revision += 1

    def _motions(self, program):
        original = json.loads(self.configuration['motion']['taught_motions_json'])
        return {'home': original['home'], **resolve_program(program, self.world_to_base)}

    def _changed(self, candidate):
        self.draft = validate_program(candidate)
        self.revision += 1
        self._clear_preview()

    def _require_manual_idle(self, recovery=False):
        if not self.manual_mode:
            raise ValueError('Select Manual Teaching first.')
        if self._cell_busy() or self.pending is not None:
            raise ValueError('Wait for the active Mission or teaching request to finish.')
        if (self.apply_uncertain or self.motion_uncertain) and not recovery:
            raise ValueError('Teaching synchronization is uncertain. Confirm idle Motion and Apply before another movement.')
        if (self.requires_restart and self.motion_instance == self.uncertain_instance):
            raise ValueError('Movement acceptance is unknown. Restart ROS, then Apply to resynchronize teaching.')

    def _measured_pose(self):
        if self.pose is None or self.clock()-self.pose_at > .5:
            raise ValueError('Fresh measured TCP feedback is unavailable.')
        return deepcopy(self.pose)

    def _request(self, operation, **payload):
        self._require_manual_idle(recovery=operation == 'apply')
        if not self.adapter.sidecar.available:
            raise ValueError('ROS teaching connection is unavailable.')
        if self.clock()-self.state_at > .5 or not self.motion_idle:
            raise ValueError('Fresh idle Motion feedback is required.')
        identity = uuid.uuid4().hex
        self.pending = dict(id=identity, operation=operation, started=self.clock(), instance=self.motion_instance)
        self.adapter.sidecar.send(dict(op='teaching_'+('move' if operation == 'move' else 'apply'),
                                      request_id=identity, **payload))
        self.detail = 'Waiting for Motion to '+('apply teaching.' if operation == 'apply' else 'complete the test movement.')

    def command(self, operation, **values):
        try:
            self._command(operation, values)
            return dict(accepted=True, detail=self.detail)
        except (ValueError, TypeError, KeyError, OSError, RuntimeError) as error:
            self.detail = str(error)
            return dict(accepted=False, detail=self.detail)

    def _command(self, operation, values):
        if operation == 'manual_mode':
            if self._cell_busy() or self.pending is not None:
                raise ValueError('Wait for the active Mission or teaching request to finish.')
            if not values['enabled'] and (self.apply_uncertain or self.motion_uncertain):
                raise ValueError('Reapply teaching to synchronize the Scene and Motion before automatic operation.')
            self.manual_mode = bool(values['enabled'])
            self.revision += 1
            self.detail = 'Manual Teaching selected; automatic Missions are suspended.' if self.manual_mode else 'Automatic operation selected.'
            return
        if operation == 'cancel':
            self.adapter.sidecar.send(dict(op='teaching_cancel'))
            self.detail = 'Cancellation requested; waiting for Motion completion.'
            return
        if operation == 'save':
            program = validate_program(self.draft)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                                 prefix='.'+self.path.name, delete=False) as stream:
                    temporary = stream.name
                    json.dump(program, stream, indent=2, allow_nan=False)
                    stream.write('\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
            finally:
                if temporary is not None and Path(temporary).exists():
                    Path(temporary).unlink()
            self.saved = deepcopy(program)
            self.revision += 1
            self.detail = 'Teaching saved. Apply to use it in this session.'
            return
        if operation == 'load':
            self._changed(json.loads(self.path.read_text(encoding='utf-8')))
            self.saved = deepcopy(self.draft)
            self.detail = 'Saved teaching loaded into the editor.'
            return
        if operation == 'revert':
            self._changed(self.active)
            self.detail = 'Editor restored to the applied teaching.'
            return
        if operation == 'edit_point':
            candidate = deepcopy(self.draft)
            if values['name'] not in candidate['points']:
                raise ValueError('Select an existing point.')
            old_point = candidate['points'][values['name']]
            point = deepcopy(values['point'])
            if 'joint_target' in old_point and motion_name(point['mode']) in JOINT_MOVEMENTS:
                point['joint_target'] = old_point['joint_target']
            else:
                point.pop('joint_target', None)
            candidate['points'][values['name']] = point
            self._changed(candidate)
        elif operation == 'point_frame':
            candidate = deepcopy(self.draft)
            pose = resolve_point(candidate, values['name'], self.world_to_base)
            candidate['points'][values['name']]['frame'] = values['frame']
            candidate['points'][values['name']]['pose'] = base_pose_to_frame(
                candidate, values['frame'], pose, self.world_to_base)
            self._changed(candidate)
        elif operation == 'add_point':
            candidate = deepcopy(self.draft)
            name = values['name'].strip()
            if not name or name in candidate['points']:
                raise ValueError('Use a new nonempty point name.')
            point = deepcopy(next(iter(candidate['points'].values())))
            candidate['points'][name] = point
            self._changed(candidate)
        elif operation == 'delete_point':
            name = values['name']
            if any(name in sequence for sequence in self.draft['sequences'].values()):
                raise ValueError('Remove the point from its sequences before deleting it.')
            candidate = deepcopy(self.draft)
            del candidate['points'][name]
            self._changed(candidate)
        elif operation == 'edit_frame':
            candidate = deepcopy(self.draft)
            candidate['frames'][values['name']]['pose'] = values['pose']
            self._changed(candidate)
        elif operation == 'edit_sequence':
            candidate = deepcopy(self.draft)
            candidate['sequences'][values['name']] = values['points']
            self._changed(candidate)
        elif operation == 'capture':
            name = values['name']
            pose = self._measured_pose()
            if (motion_name(self.draft['points'][name]['mode']) in JOINT_MOVEMENTS
                    and self.joint_positions is None):
                raise ValueError('Measured arm joint positions for this TCP sample are unavailable.')
            self._changed(capture_point(self.draft, name, pose, self.world_to_base, self.joint_positions))
        elif operation == 'read_scene':
            self._changed(self._scene_program())
        elif operation == 'preview':
            self._preview(values['name'])
            self.detail = 'Target axes and nominal waypoint links shown; test movement validates the path.'
            return
        elif operation == 'apply':
            self._require_manual_idle(recovery=True)
            candidate = validate_program(self.draft)
            if candidate['frames'] != self.active['frames']:
                if not self.adapter.timeline.is_stopped() or self.adapter.conveyor.active_roll_paths:
                    raise ValueError('Stop Timeline and clear supplied material before applying equipment placement.')
            self._request('apply', motions=self._motions(candidate))
            self._apply_snapshot = candidate
            return
        elif operation == 'move':
            point = self.draft['points'][values['name']]
            motion = dict(cartesian_waypoints=[resolve_waypoint(self.draft, values['name'], self.world_to_base)],
                          velocity_scale=min(.1, point['velocity_scale']), acceleration_scale=min(.1, point['acceleration_scale']))
            self._request('move', motion=motion)
            return
        elif operation == 'jog':
            axis, step, frame = values['axis'], values['step'], values['frame']
            if isinstance(axis, bool) or axis not in range(6) or isinstance(step, bool) or not isinstance(step, (int, float)) or not math.isfinite(step):
                raise ValueError('Jog requires a finite signed step and axis 0 through 5.')
            if not 0 < abs(step) <= (.01 if axis < 3 else math.radians(5)):
                raise ValueError('Jog step limit is 10 mm or 5 degrees.')
            measured = self._measured_pose()
            if frame == 'tool':
                delta = [0.]*6
                delta[axis] = step
                target = matrix_pose7(pose7_matrix(measured) @ pose_matrix(delta))
            else:
                local = base_pose_to_frame(self.draft, frame, measured, self.world_to_base)
                if axis < 3:
                    local[axis] += step
                    target = frame_pose_to_base(self.draft, frame, local, self.world_to_base)
                else:
                    # Rotate about the selected frame's axis while keeping TCP position fixed.
                    delta = [0.]*6
                    delta[axis] = step
                    rotation = pose_matrix(delta)[:3, :3] @ pose_matrix(local)[:3, :3]
                    local_matrix = pose_matrix(local)
                    local_matrix[:3, :3] = rotation
                    from .teaching_program import matrix_pose
                    target = frame_pose_to_base(self.draft, frame, matrix_pose(local_matrix), self.world_to_base)
            self._request('move', motion=dict(cartesian_waypoints=[dict(pose=target, motion='MoveL')],
                                              velocity_scale=.05, acceleration_scale=.05))
            return
        else:
            raise ValueError('Unknown teaching operation: '+str(operation))
        self.detail = 'Draft updated. Save persists it; Apply selects it for automatic operation.'

    def _scene_program(self):
        from pxr import UsdGeom
        from .teaching_program import matrix_pose
        candidate = deepcopy(self.draft)
        cache = UsdGeom.XformCache()
        references = self.configuration['cell']['tape_roll']['equipment_reference_frames']
        for name in ('infeed', 'outfeed'):
            prim = self.adapter.stage.GetPrimAtPath('/World/TapeRollCell/'+name.capitalize())
            if not prim:
                raise ValueError('Equipment frame is unavailable in the Scene: '+name)
            world = np.asarray(cache.GetLocalToWorldTransform(prim)).T
            candidate['frames'][name]['pose'] = matrix_pose(world @ pose_matrix(references[name]['pose']))
        return candidate

    def _clear_preview(self):
        stage = self.adapter.stage
        if stage is not None and stage.GetPrimAtPath(self.preview_path):
            stage.RemovePrim(self.preview_path)

    def _preview(self, name):
        from pxr import Gf, UsdGeom
        self._clear_preview()
        stage = self.adapter.stage
        target = base_matrix(self.world_to_base) @ pose7_matrix(resolve_point(self.draft, name, self.world_to_base))
        origin = target[:3, 3]
        axes = UsdGeom.BasisCurves.Define(stage, self.preview_path+'/TCPAxes')
        vertices = []
        for axis in range(3):
            vertices.extend([Gf.Vec3f(*origin), Gf.Vec3f(*(origin + .08*target[:3, axis]))])
        axes.CreatePointsAttr(vertices)
        axes.CreateCurveVertexCountsAttr([2, 2, 2])
        axes.CreateTypeAttr('linear')
        axes.CreateWidthsAttr([.003])
        axes.CreateDisplayColorAttr([(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)])
        axes.GetDisplayColorPrimvar().SetInterpolation('uniform')
        for sequence, names in self.draft['sequences'].items():
            if len(names) < 2:
                continue
            line = UsdGeom.BasisCurves.Define(stage, self.preview_path+'/'+sequence)
            positions = [(base_matrix(self.world_to_base) @ pose7_matrix(resolve_point(self.draft, point, self.world_to_base)))[:3, 3] for point in names]
            line.CreatePointsAttr([Gf.Vec3f(*position) for position in positions])
            line.CreateCurveVertexCountsAttr([len(positions)])
            line.CreateTypeAttr('linear')
            line.CreateWidthsAttr([.002])
            line.CreateDisplayColorAttr([(.9, .8, .2)])

    def close(self):
        if self.pending is not None:
            self.adapter.sidecar.send(dict(op='teaching_cancel'))
        from isaac_runtime.cell_teaching import clear
        clear(self)
        self._clear_preview()

    def tick(self):
        if self.pending is None:
            return
        maximum = 15. if self.pending['operation'] == 'apply' else float('inf')
        if self.adapter.sidecar.available and self.clock()-self.pending['started'] <= maximum:
            return
        packet = dict(op='teaching_result', request_id=self.pending['id'], success=False,
                      uncertain=True, requires_restart=self.pending['operation'] == 'move',
                      detail='Teaching connection or response was lost. Save edits; restore ROS and Apply before resuming.')
        if self.pending['operation'] == 'move':
            self.adapter.sidecar.send(dict(op='teaching_cancel'))
        self.consume(packet)

    def on_stop(self):
        self.pose_at = self.state_at = float('-inf')
        if self.pending is not None and self.pending['operation'] == 'move':
            self.adapter.sidecar.send(dict(op='teaching_cancel'))
            self.detail = 'Timeline stopped; waiting for manual movement cancellation.'
