"""ROS-side teaching requests; all movement admission remains in Motion."""

import json
import math
import time


class TeachingRosBridge:
    def __init__(self, node, send):
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from moveit_msgs.srv import GetPositionFK
        from arm_cell_interfaces.msg import MotionStatus
        from arm_cell_interfaces.action import ExecuteTeaching
        from arm_cell_interfaces.srv import ConfigureTeaching

        self.node, self.send = node, send
        self.action = ActionClient(node, ExecuteTeaching, '/motion/execute_teaching')
        self.configure = node.create_client(ConfigureTeaching, '/motion/configure_teaching')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.joints = None
        self.arm_joint_names = ()
        self.joints_at = self.motion_at = self.pose_at = 0.
        self.motion = None
        self.pose = None
        self.query = None
        self.query_at = 0.
        self.goal = None
        self.pending = False
        self.cancel_requested = False
        self.operation = None
        self.motion_instance = ''
        node.create_subscription(JointState, '/joint_states', self._joints, 10)
        node.create_subscription(MotionStatus, '/motion/state', self._motion, 10)

    def _joints(self, message):
        if (not self.arm_joint_names or len(message.name) != len(message.position) or
                len(set(message.name)) != len(message.name) or
                not set(self.arm_joint_names).issubset(message.name) or
                not all(math.isfinite(value) for value in message.position)):
            self._invalidate_observation()
            return
        self.joints, self.joints_at = message, time.monotonic()

    def _invalidate_observation(self):
        self.joints = None
        self.pose_at = float('-inf')
        query = self.query
        self.query = None
        if query is not None:
            query.cancel()

    def _motion(self, message):
        self.motion, self.motion_at = message, time.monotonic()
        # Humble rclpy subscriptions supply only the message, not MessageInfo.
        # A unique graph endpoint identifies the Motion process across restart.
        try:
            publishers = self.node.get_publishers_info_by_topic('/motion/state')
            self.motion_instance = bytes(publishers[0].endpoint_gid).hex() if len(publishers) == 1 else ''
            if not self.motion_instance:
                self.motion_at = float('-inf')
        except Exception:
            self.motion_instance = ''
            self.motion_at = float('-inf')

    def _result(self, identity, success, detail, uncertain=False, requires_restart=False):
        self.send(dict(op='teaching_result', request_id=identity,
                       success=bool(success), detail=str(detail), uncertain=uncertain,
                       requires_restart=requires_restart))

    def _begin(self, identity, kind):
        self.pending = True
        self.operation = dict(id=identity, kind=kind, started=time.monotonic(), accepted=False)

    def _current(self, identity):
        return self.operation is not None and self.operation['id'] == identity

    def _finish(self, identity, success, detail):
        if self._current(identity):
            self.pending, self.goal, self.operation = False, None, None
            self._result(identity, success, detail)

    def _cancel_goal(self):
        if self.goal is not None:
            try:
                self.goal.cancel_goal_async()
            except Exception:
                # Keep cancellation best effort across ROS endpoint loss;
                # uncertain outcomes still require idle configuration recovery.
                pass

    def command(self, packet, mission_busy=False):
        from arm_cell_interfaces.action import ExecuteTeaching
        from arm_cell_interfaces.srv import ConfigureTeaching

        operation, identity = packet['op'], packet.get('request_id')
        if operation == 'teaching_settings':
            names = packet.get('arm_joint_names')
            if (isinstance(names, list) and names and len(set(names)) == len(names) and
                    all(isinstance(name, str) and name for name in names)):
                self.arm_joint_names = tuple(names)
                self._invalidate_observation()
            return
        if operation == 'teaching_cancel':
            self.cancel_requested = True
            self._cancel_goal()
            return
        if self.pending or self.goal is not None or mission_busy:
            self._result(identity, False, 'A teaching request or Mission is already active.')
            return
        if operation == 'teaching_apply':
            if not self.configure.service_is_ready():
                self._result(identity, False, 'Motion teaching configuration service is unavailable.')
                return
            self._begin(identity, 'apply')
            request = ConfigureTeaching.Request()
            request.taught_motions_json = json.dumps(packet['motions'], allow_nan=False)
            try:
                future = self.configure.call_async(request)
            except Exception as error:
                self.pending, self.operation = False, None
                self._result(identity, False, str(error), uncertain=True)
                return

            def configured(future):
                if not self._current(identity):
                    return
                try:
                    response = future.result()
                    self._finish(identity, response.success, response.diagnostic_detail)
                except Exception as error:
                    self.operation = None
                    self.pending = False
                    self._result(identity, False, str(error), uncertain=True)
            future.add_done_callback(configured)
        elif operation == 'teaching_move':
            if not self.action.server_is_ready():
                self._result(identity, False, 'Motion teaching action is unavailable.')
                return
            self._begin(identity, 'move')
            self.cancel_requested = False
            goal = ExecuteTeaching.Goal()
            goal.motion_json = json.dumps(packet['motion'], allow_nan=False)
            try:
                future = self.action.send_goal_async(goal)
            except Exception as error:
                self.pending, self.operation = False, None
                self._result(identity, False, str(error), uncertain=True, requires_restart=True)
                return

            def accepted(future):
                try:
                    goal_handle = future.result()
                    if not self._current(identity):
                        if goal_handle is not None and goal_handle.accepted:
                            goal_handle.cancel_goal_async()
                        return
                    self.goal = goal_handle
                    if self.goal is None or not self.goal.accepted:
                        self._finish(identity, False, 'Motion rejected the teaching movement.')
                        return
                    self.operation['accepted'] = True
                    if self.cancel_requested:
                        self._cancel_goal()
                    self.goal.get_result_async().add_done_callback(finished)
                except Exception as error:
                    if self._current(identity):
                        self._cancel_goal()
                        self.pending, self.goal, self.operation = False, None, None
                        self._result(identity, False, str(error), uncertain=True, requires_restart=True)

            def finished(future):
                if not self._current(identity):
                    return
                try:
                    from arm_cell_interfaces.msg import MotionTaskResultCode
                    response = future.result()
                    success = (response.status == 4 and
                               response.result.result_code == MotionTaskResultCode.TASK_RESULT_SUCCESS)
                    self._finish(identity, success, response.result.diagnostic_detail)
                except Exception as error:
                    self._cancel_goal()
                    self.pending, self.goal, self.operation = False, None, None
                    self._result(identity, False, str(error), uncertain=True)
            future.add_done_callback(accepted)

    def publish(self):
        from moveit_msgs.srv import GetPositionFK
        now = time.monotonic()
        operation = self.operation
        if operation is not None:
            elapsed = now-operation['started']
            endpoint_ready = (self.configure.service_is_ready() if operation['kind'] == 'apply'
                              else self.action.server_is_ready())
            deadline = float('inf') if operation['accepted'] else 10.
            terminal = self.goal is not None and self.goal.status in (4, 5, 6)
            if terminal:
                operation.setdefault('terminal_at', now)
            result_lost = terminal and now-operation['terminal_at'] > 10.
            feedback_lost = (operation['accepted'] and now-self.motion_at > 2. and elapsed > 2.)
            if elapsed > deadline or result_lost or feedback_lost or (elapsed > 1. and not endpoint_ready):
                self._cancel_goal()
                self.cancel_requested = True
                self.pending, self.goal, self.operation = False, None, None
                self._result(operation['id'], False,
                             'Teaching response unavailable. Confirm idle Motion and reapply; restart ROS if acceptance was unknown.',
                             uncertain=True, requires_restart=operation['kind'] == 'move' and not operation['accepted'])
        # Consume FK on the timer's mutually-exclusive callback group. Future
        # done callbacks run independently in a MultiThreadedExecutor and could
        # otherwise restore a pose after joint feedback has invalidated it.
        if self.query is not None and self.query.done():
            query = self.query
            self.query = None
            try:
                response = query.result()
                if response.error_code.val == 1 and len(response.pose_stamped) == 1:
                    value = response.pose_stamped[0].pose
                    self.pose = [value.position.x, value.position.y, value.position.z,
                                 value.orientation.x, value.orientation.y,
                                 value.orientation.z, value.orientation.w]
                    self.pose_at = self.query_observation_at
                else:
                    self.pose_at = float('-inf')
            except Exception:
                self.pose_at = float('-inf')
        if self.query is not None and now-self.query_at > 1.:
            expired = self.query
            self.query = None
            expired.cancel()
            self.pose_at = float('-inf')
        if (self.query is None and self.joints is not None and
                now - self.joints_at <= .5 and self.fk.service_is_ready()):
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['sf_grasp_tcp']
            request.robot_state.joint_state = self.joints
            request.robot_state.is_diff = True
            self.query_observation_at = self.joints_at
            try:
                self.query = self.fk.call_async(request)
            except Exception:
                self.query = None
                self.pose_at = float('-inf')
            self.query_at = now

        idle = bool(self.motion is not None and now-self.motion_at <= .5 and
                    not self.motion.has_active_execution and not self.motion.execution_active and
                    self.motion.backend_inactivity_confirmed and self.motion.execution_state in (1, 4))
        self.send(dict(op='teaching_state', current_pose=self.pose,
                       pose_valid=self.pose is not None and now-self.pose_at <= .5,
                       motion_idle=idle, motion_ready=self.action.server_is_ready(),
                       motion_instance=self.motion_instance,
                       busy=self.pending or self.goal is not None))

    def close(self):
        self.cancel_requested = True
        self._cancel_goal()
