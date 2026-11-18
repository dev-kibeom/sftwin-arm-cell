"""Operator teaching controls; all operations are delegated to the cell owner."""
from __future__ import annotations

import math


_AXES = ('X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)')
_POINT_FRAMES = ('base', 'infeed', 'outfeed')


def display_pose(pose):
    """Convert the owner's metres/radians to operator millimetres/degrees."""
    return [value * 1000 if index < 3 else math.degrees(value)
            for index, value in enumerate(pose)]


def owner_pose(pose):
    return [value / 1000 if index < 3 else math.radians(value)
            for index, value in enumerate(pose)]


class TeachingPanel:
    def __init__(self, ui):
        self._ui = ui
        self._port = None
        self._status = None
        self._syncing = False
        self._waiting_revision = None
        self._choices = {}
        self._combos = {}
        self._selector_frames = {}
        self._controls = []
        self._edit_fields = []
        self._message = ''
        self._built = False

    def build(self):
        from isaac_runtime import cell_teaching

        editor = self._editor_snapshot() if self._built else None
        self.close()
        self._port = cell_teaching
        self._status = self._port.status()
        if self._status is None:
            return
        ui = self._ui
        with ui.CollapsableFrame('Teaching', collapsed=True):
            with ui.VStack(spacing=4):
                self._summary = ui.Label('', word_wrap=True)
                self._feedback = ui.Label('', word_wrap=True)
                self._pose_label = ui.Label('', word_wrap=True)
                with ui.HStack(height=24):
                    self._manual = ui.CheckBox(width=24)
                    ui.Label('Manual Teaching')
                self._manual.model.add_value_changed_fn(self._manual_changed)
                ui.Label('Update point before a test move. Save persists edits; Apply selects automatic operation.', word_wrap=True)
                with ui.HStack(height=26):
                    for label, operation in (('Load', 'load'), ('Save', 'save'),
                                             ('Apply', 'apply'), ('Revert', 'revert')):
                        self._button(label, lambda op=operation: self._command(op),
                                     'apply' if operation == 'apply' else 'edit')
                with ui.CollapsableFrame('Taught points', collapsed=False):
                    with ui.VStack(spacing=4):
                        self._selector('point', 'Point', self._point_selected)
                        with ui.HStack(height=26):
                            ui.Label('New point name', width=110)
                            self._new_name = ui.StringField()
                            self._edit_fields.append(self._new_name)
                            self._button('Add', self._add_point, 'edit')
                            self._button('Delete', lambda: self._point_command('delete_point'), 'point')
                        self._selector('point_frame', 'Point reference frame', self._point_frame_changed,
                                       list(_POINT_FRAMES))
                        self._point_pose = self._pose_fields()
                        self._selector('mode', 'Motion', None, ['joint', 'linear'])
                        with ui.HStack(height=26):
                            ui.Label('Speed (%)', width=100)
                            self._speed = ui.FloatField()
                            ui.Label('Acceleration (%)', width=120)
                            self._acceleration = ui.FloatField()
                            self._edit_fields.extend((self._speed, self._acceleration))
                        with ui.HStack(height=26):
                            self._button('Update point', self._edit_point, 'point')
                            self._button('Capture current TCP', lambda: self._point_command('capture'), 'capture')
                        with ui.HStack(height=26):
                            self._button('Preview', lambda: self._point_command('preview'), 'point')
                            self._button('Test move', lambda: self._point_command('move'), 'move')
                            self._button('Stop', lambda: self._command('cancel'), 'stop')
                with ui.CollapsableFrame('TCP step jog', collapsed=True):
                    with ui.VStack(spacing=4):
                        self._selector('jog_frame', 'Jog reference frame', None,
                                       ['base', 'infeed', 'outfeed', 'tool'])
                        with ui.HStack(height=26):
                            ui.Label('Step (mm)', width=100)
                            self._linear_step = ui.FloatField()
                            self._linear_step.model.set_value(1.0)
                            ui.Label('Step (deg)', width=100)
                            self._angular_step = ui.FloatField()
                            self._angular_step.model.set_value(1.0)
                            self._edit_fields.extend((self._linear_step, self._angular_step))
                        for axis, label in enumerate(_AXES):
                            with ui.HStack(height=26):
                                ui.Label(label, width=110)
                                self._button('-', lambda a=axis: self._jog(a, -1), 'pose')
                                self._button('+', lambda a=axis: self._jog(a, 1), 'pose')
                with ui.CollapsableFrame('Equipment frames', collapsed=True):
                    with ui.VStack(spacing=4):
                        ui.Label('Frame pose relative to world', word_wrap=True)
                        self._selector('equipment', 'Equipment', self._frame_selected,
                                       ['infeed', 'outfeed'])
                        self._frame_pose = self._pose_fields()
                        with ui.HStack(height=26):
                            self._button('Update frame', self._edit_frame, 'edit')
                            self._button('Read Scene Layout', lambda: self._command('read_scene'), 'edit')
                with ui.CollapsableFrame('Sequences', collapsed=True):
                    with ui.VStack(spacing=4):
                        self._selector('sequence', 'Sequence', self._sequence_selected)
                        self._sequence_text = ui.Label('', word_wrap=True)
                        self._selector('sequence_step', 'Selected step', None)
                        with ui.HStack(height=26):
                            self._button('Append selected point', lambda: self._edit_sequence('append'), 'point')
                            self._button('Remove step', lambda: self._edit_sequence('remove'), 'edit')
                        with ui.HStack(height=26):
                            self._button('Move step up', lambda: self._edit_sequence('up'), 'edit')
                            self._button('Move step down', lambda: self._edit_sequence('down'), 'edit')
        self._built = True
        self._sync_editors()
        if editor is not None:
            self._restore_editor(editor)
        self.refresh()

    def _editor_snapshot(self):
        return dict(point=self._selected('point'), equipment=self._selected('equipment'),
                    point_pose=self._get_pose(self._point_pose), frame_pose=self._get_pose(self._frame_pose),
                    point_frame=self._selected('point_frame'), mode=self._selected('mode'),
                    speed=self._speed.model.as_float, acceleration=self._acceleration.model.as_float,
                    jog_frame=self._selected('jog_frame'), linear_step=self._linear_step.model.as_float,
                    angular_step=self._angular_step.model.as_float, new_name=self._new_name.model.as_string)

    def _restore_editor(self, editor):
        self._syncing = True
        try:
            for key in ('point', 'equipment', 'point_frame', 'mode', 'jog_frame'):
                if editor[key] in self._choices[key]:
                    self._combos[key].model.get_item_value_model().set_value(self._choices[key].index(editor[key]))
            self._set_pose(self._point_pose, editor['point_pose'])
            self._set_pose(self._frame_pose, editor['frame_pose'])
            self._speed.model.set_value(editor['speed'])
            self._acceleration.model.set_value(editor['acceleration'])
            self._linear_step.model.set_value(editor['linear_step'])
            self._angular_step.model.set_value(editor['angular_step'])
            self._new_name.model.set_value(editor['new_name'])
        finally:
            self._syncing = False

    def _button(self, label, callback, requirement):
        button = self._ui.Button(label, clicked_fn=callback)
        self._controls.append((button, requirement))
        return button

    def _selector(self, key, label, changed, values=()):
        with self._ui.HStack(height=26):
            self._ui.Label(label, width=150)
            self._selector_frames[key] = self._ui.Frame()
        self._set_choices(key, list(values), changed=changed)

    def _set_choices(self, key, values, selected=None, changed=None):
        if selected is None:
            selected = self._selected(key)
        self._choices[key] = values
        frame = self._selector_frames[key]
        frame.clear()
        with frame:
            combo = self._ui.ComboBox(values.index(selected) if selected in values else 0,
                                      *(values or ['(none)']))
        self._combos[key] = combo
        if changed:
            combo.model.add_item_changed_fn(lambda *_: None if self._syncing else changed())

    def _selected(self, key):
        values = self._choices.get(key, [])
        if not values:
            return None
        index = self._combos[key].model.get_item_value_model().as_int
        return values[index] if 0 <= index < len(values) else None

    def _pose_fields(self):
        fields = []
        for start in (0, 3):
            with self._ui.HStack(height=26):
                for index in range(start, start + 3):
                    self._ui.Label(_AXES[index], width=75)
                    field = self._ui.FloatField()
                    fields.append(field)
                    self._edit_fields.append(field)
        return fields

    @staticmethod
    def _set_pose(fields, pose):
        for field, value in zip(fields, display_pose(pose)):
            field.model.set_value(value)

    @staticmethod
    def _get_pose(fields):
        return owner_pose([field.model.as_float for field in fields])

    def _program(self):
        return (self._status or {}).get('program', {})

    def _sync_editors(self):
        if not self._built or self._status is None:
            return
        self._syncing = True
        try:
            self._set_choices('point', list(self._program().get('points', {})),
                              changed=self._point_selected)
            self._set_choices('sequence', list(self._program().get('sequences', {})),
                              changed=self._sequence_selected)
            self._point_selected()
            self._frame_selected()
            self._sequence_selected()
        finally:
            self._syncing = False

    def _point_selected(self):
        point = self._program().get('points', {}).get(self._selected('point'))
        if point is None:
            return
        self._set_pose(self._point_pose, point['pose'])
        self._set_choices('point_frame', list(_POINT_FRAMES), point['frame'], self._point_frame_changed)
        self._set_choices('mode', ['joint', 'linear'], point['mode'])
        self._speed.model.set_value(point['velocity_scale'] * 100)
        self._acceleration.model.set_value(point['acceleration_scale'] * 100)

    def _frame_selected(self):
        frame = self._program().get('frames', {}).get(self._selected('equipment'))
        if frame:
            self._set_pose(self._frame_pose, frame['pose'])

    def _sequence_selected(self):
        points = self._program().get('sequences', {}).get(self._selected('sequence'), [])
        self._sequence_text.text = ' -> '.join(points) if points else 'No steps'
        self._set_choices('sequence_step', [f'{i + 1}: {name}' for i, name in enumerate(points)])

    def _manual_changed(self, model):
        if not self._syncing:
            self._command('manual_mode', enabled=model.as_bool)

    def _point_frame_changed(self):
        self._point_command('point_frame', frame=self._selected('point_frame'))

    def _add_point(self):
        name = self._new_name.model.as_string.strip()
        if self._command('add_point', name=name):
            self._set_choices('point', list(self._program().get('points', {})), name,
                              self._point_selected)
            self._point_selected()

    def _point_command(self, operation, **kwargs):
        name = self._selected('point')
        if name:
            return self._command(operation, name=name, **kwargs)
        return False

    def _edit_point(self):
        self._point_command('edit_point', point={
            'frame': self._selected('point_frame'), 'pose': self._get_pose(self._point_pose),
            'mode': self._selected('mode'), 'velocity_scale': self._speed.model.as_float / 100,
            'acceleration_scale': self._acceleration.model.as_float / 100,
        })

    def _edit_frame(self):
        self._command('edit_frame', name=self._selected('equipment'),
                      pose=self._get_pose(self._frame_pose))

    def _jog(self, axis, sign):
        step = self._linear_step.model.as_float if axis < 3 else self._angular_step.model.as_float
        if not math.isfinite(step) or step <= 0:
            self._message = 'Jog step must be a positive, finite number.'
            return
        self._command('jog', frame=self._selected('jog_frame'), axis=axis,
                      step=sign * (step / 1000 if axis < 3 else math.radians(step)))

    def _edit_sequence(self, operation):
        name = self._selected('sequence')
        if name is None:
            self._message = 'Select a sequence first.'
            return
        points = list(self._program()['sequences'][name])
        selected = self._selected('sequence_step')
        index = self._choices['sequence_step'].index(selected) if selected else -1
        if operation == 'append':
            point = self._selected('point')
            if point is None:
                return
            points.append(point)
        elif index < 0:
            return
        elif operation == 'remove':
            points.pop(index)
        else:
            destination = index + (-1 if operation == 'up' else 1)
            if not 0 <= destination < len(points):
                return
            points[index], points[destination] = points[destination], points[index]
        self._command('edit_sequence', name=name, points=points)

    def _command(self, operation, **kwargs):
        if self._port is None:
            return False
        try:
            result = self._port.command(operation, **kwargs)
            accepted = bool(result.get('accepted'))
            self._message = result.get('detail') or ('Accepted' if accepted else 'Request rejected')
            if accepted and operation in {
                'add_point', 'delete_point', 'point_frame', 'edit_point', 'capture',
                'edit_frame', 'read_scene', 'edit_sequence', 'load', 'revert',
            }:
                self._waiting_revision = (self._status or {}).get('revision')
            self.refresh()
            return accepted
        except (RuntimeError, ValueError, KeyError) as error:
            self._message = str(error)
            return False

    def refresh(self):
        if not self._built or self._port is None:
            return
        self._status = self._port.status()
        status = self._status or {}
        available = self._status is not None
        pending = bool(status.get('pending'))
        # Normal polling never replaces draft fields. Only an accepted command
        # that changes the program authorizes loading new values into editors.
        if self._waiting_revision is not None and (
                status.get('revision') != self._waiting_revision or not pending):
            self._waiting_revision = None
            self._sync_editors()
        manual = bool(status.get('manual_mode'))
        fresh = bool(status.get('current_pose_valid')) and status.get('current_pose') is not None
        editable = available
        movable = available and manual and not pending
        point = self._selected('point') is not None
        enabled = dict(apply=movable, move=movable and fresh and point,
                       pose=movable and fresh, capture=available and fresh and point,
                       point=available and point, edit=available, stop=available)
        for widget, requirement in self._controls:
            widget.enabled = enabled[requirement]
        for field in self._edit_fields:
            field.enabled = editable
        self._combos['point_frame'].enabled = editable and point
        self._combos['mode'].enabled = editable and point
        self._manual.enabled = available and not pending
        self._syncing = True
        try:
            self._manual.model.set_value(manual)
        finally:
            self._syncing = False
        state = ('Unavailable' if not available else 'Moving / request pending' if pending else
                 'Manual Teaching enabled' if manual else 'Manual Teaching disabled')
        self._summary.text = '\n'.join(filter(None, [state,
            'Unsaved changes' if status.get('dirty') else 'Saved',
            'Changes not applied to runtime' if status.get('unapplied') else 'Applied',
            str(status.get('path', '')), str(status.get('detail', ''))]))
        self._feedback.text = self._message
        pose = status.get('current_pose6')
        self._pose_label.text = ('Current TCP (base): ' + ', '.join(
            f'{axis}={value:.2f}' for axis, value in zip(_AXES, display_pose(pose)))
            if fresh and pose is not None else 'Current TCP: waiting for a fresh pose')
    def close(self):
        self._built = False
        self._port = None
        self._status = None
        self._controls.clear()
        self._edit_fields.clear()
        self._combos.clear()
        self._selector_frames.clear()
        self._choices.clear()
        self._waiting_revision = None
