"""ROS ABI boundary: Kit JSON client and system-Python configured Supply server."""
import json
import os
from pathlib import Path
import select
import shlex
import socket
import subprocess
import sys
import time


def supply_announced_for_mission(state, fresh, published_epoch):
    """Advertise admission only after this valid epoch has been published.

    The consumer still owns admission. A new docking observation must cross
    the Cell state topic before its corresponding Mission request is sent.
    """
    return bool(fresh and state and state.get('valid') and state.get('supply_epoch')
                and state['supply_epoch']==published_epoch)


class ConfiguredSupplyClient:
    def __init__(self, sock, process=None):
        self.socket, self.process = sock, process
        self.socket.setblocking(False)
        self.buffer = bytearray()
        self.outgoing = []
        self.available = True

    @classmethod
    def launch(cls):
        root = Path(os.environ['SFTWIN_PROJECT_ROOT'])
        setup = Path(os.environ.get('ARM_CELL_ROS_SETUP', root/'ros2_ws/install/setup.bash'))
        if not setup.is_file():
            raise RuntimeError(f'Configured Supply requires the ROS overlay: {setup}')
        parent, child = socket.socketpair()
        env = os.environ.copy()
        env.pop('PYTHONHOME', None)
        env['PYTHONPATH'] = ''
        command = ('source /opt/ros/humble/setup.bash && source '+shlex.quote(str(setup))+
                   ' && exec /usr/bin/python3.10 '+shlex.quote(str(Path(__file__).resolve()))+
                   ' '+str(child.fileno()))
        try:
            process = subprocess.Popen(['/bin/bash','-c',command], env=env,
                pass_fds=(child.fileno(),), close_fds=True, stdout=subprocess.DEVNULL, stderr=sys.stderr)
        except Exception:
            parent.close()
            child.close()
            raise
        child.close()
        client = cls(parent, process)
        deadline = time.monotonic()+15.
        while time.monotonic() < deadline:
            for message in client.poll_messages():
                if message.get('op') == 'ready':
                    if message.get('ready'):
                        return client
                    client.close()
                    raise RuntimeError(message.get('error', 'Configured Supply sidecar initialization failed'))
            if not client.available or process.poll() is not None:
                break
            select.select([parent], [], [], .1)
        client.close()
        raise RuntimeError('Configured Supply sidecar startup timed out')

    def poll_messages(self):
        messages = []
        if self.process is not None and self.process.poll() is not None:
            self.available = False
        while self.available:
            try:
                chunk = self.socket.recv(65536)
            except BlockingIOError:
                break
            except OSError:
                self.available = False
                break
            if not chunk:
                self.available = False
                break
            self.buffer.extend(chunk)
        while b'\n' in self.buffer:
            line, _, remainder = self.buffer.partition(b'\n')
            self.buffer = bytearray(remainder)
            try:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError('Non-object IPC packet')
                messages.append(packet)
            except (ValueError, UnicodeDecodeError):
                self.available = False
                break
        return messages

    def send(self, packet):
        if self.available:
            self.outgoing.append(memoryview((json.dumps(packet, allow_nan=False)+'\n').encode()))
            self.flush_outgoing()

    def flush_outgoing(self):
        while self.available and self.outgoing:
            try:
                sent = self.socket.send(self.outgoing[0])
                if sent <= 0:
                    self.available = False
                    break
                self.outgoing[0] = self.outgoing[0][sent:]
                if not self.outgoing[0]: self.outgoing.pop(0)
            except BlockingIOError:
                break
            except OSError:
                self.available = False

    def close(self):
        self.available = False
        self.socket.close()
        if self.process is not None:
            try:
                self.process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()


def serve(fd):
    import threading
    import uuid
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from arm_cell_interfaces.srv import ConfiguredSupply
    from arm_cell_interfaces.msg import ConfiguredSupplyState
    sock = socket.socket(fileno=fd)
    send_lock = threading.Lock()
    condition = threading.Condition()
    replies = {}
    pending = set()
    operator_requests = []
    cache = {'state':None, 'received_at':0., 'connected':True}
    def send(packet):
        with send_lock:
            sock.sendall((json.dumps(packet, allow_nan=False)+'\n').encode())
    def reader():
        try:
            with sock.makefile('rb') as stream:
                for raw in stream:
                    packet = json.loads(raw)
                    with condition:
                        if packet.get('op') == 'response':
                            identity = packet.get('correlation_id')
                            if identity in pending:
                                replies[identity] = packet['response']
                        elif packet.get('op') in ('start_cycle','cancel_cycle',
                                                  'teaching_apply','teaching_move','teaching_cancel','teaching_settings'):
                            operator_requests.append(packet)
                        elif packet.get('op') == 'state':
                            cache.update(state=packet['state'], received_at=time.monotonic())
                        condition.notify_all()
        except (OSError, ValueError, KeyError):
            pass
        finally:
            with condition:
                cache['connected'] = False
                condition.notify_all()
    rclpy.init()
    node = rclpy.create_node('configured_supply_sidecar')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
    publisher = node.create_publisher(ConfiguredSupplyState, '/cell/configured_supply/state', 10)
    from rclpy.action import ActionClient
    from arm_cell_interfaces.action import ExecuteCycle
    cycle = ActionClient(node, ExecuteCycle, '/orchestration/execute_cycle')
    active_cycle = {'goal':None,'pending':False,'request_id':None,'cancel':False}
    from tape_roll_cell.teaching_ros import TeachingRosBridge
    teaching = TeachingRosBridge(node, send)
    def cycle_response(future):
        goal = future.result();active_cycle['pending']=False
        if goal is None or not goal.accepted:
            send(dict(op='mission_result',request_id=active_cycle['request_id'],success=False,detail='Mission rejected: check ROS, Safety, and supply readiness.'));return
        active_cycle['goal']=goal
        if active_cycle['cancel']:goal.cancel_goal_async()
        result = goal.get_result_async()
        def finished(future):
            outcome=future.result();active_cycle['goal']=None
            send(dict(op='mission_result',request_id=active_cycle['request_id'],
                success=outcome.status==4 and outcome.result.exit_reason.value==7,detail=outcome.result.diagnostic_detail))
            send(dict(op='mission_status',text='Mission ended: '+outcome.result.diagnostic_detail))
        result.add_done_callback(finished)
    def service(request, response):
        identity = uuid.uuid4().hex
        payload = {name:getattr(request,name) for name in request.get_fields_and_field_types()}
        deadline = time.monotonic()+3.
        with condition:
            pending.add(identity)
        try:
            send(dict(op='request', correlation_id=identity, request=payload, deadline_monotonic=deadline))
            with condition:
                while identity not in replies and cache['connected']:
                    remaining = deadline-time.monotonic()
                    if remaining <= 0: break
                    condition.wait(remaining)
                result = replies.pop(identity, None)
            if result is None:
                response.success = False
                response.diagnostic_detail = 'isaac_owner_unavailable_or_timeout'
            else:
                for name in response.get_fields_and_field_types():
                    if name in result: setattr(response,name,result[name])
        except OSError:
            response.success = False
            response.diagnostic_detail = 'isaac_owner_disconnected'
        finally:
            with condition:
                pending.discard(identity)
        return response
    from arm_cell_interfaces.msg import SafetyState
    safety={'state':None,'at':0.}
    def safety_received(message):safety.update(state=message,at=time.monotonic())
    node.create_subscription(SafetyState,'/safety/state',safety_received,10)
    node.create_service(ConfiguredSupply, '/cell/configured_supply', service,
                        callback_group=ReentrantCallbackGroup())
    published_supply={'epoch':None}
    def publish():
        with condition:
            announced=cache['state']
            fresh=cache['connected'] and time.monotonic()-cache['received_at']<=.5
        safe=safety['state']
        available=(safe is not None and time.monotonic()-safety['at']<.5 and safe.valid and
            safe.required_inputs_fresh and safe.motion_envelope_valid and safe.motion_capability.value==2 and
            safe.max_velocity_scale>0 and safe.max_acceleration_scale>0 and cycle.server_is_ready() and
            active_cycle['goal'] is None and not active_cycle['pending'] and
            supply_announced_for_mission(announced,fresh,published_supply['epoch']))
        send(dict(op='mission_available',ready=available))
        with condition:
            commands=list(operator_requests);operator_requests.clear()
        for packet in commands:
            if packet.get('op', '').startswith('teaching_'):
                teaching.command(packet, mission_busy=active_cycle['goal'] is not None or active_cycle['pending'])
                continue
            if packet.get('op')=='cancel_cycle':
                active_cycle['cancel']=True
                if active_cycle['goal'] is not None:active_cycle['goal'].cancel_goal_async()
                continue
            if active_cycle['goal'] is not None or active_cycle['pending'] or not cycle.server_is_ready():
                send(dict(op='mission_result',request_id=packet.get('request_id'),success=False,detail='A Mission is already running or ROS is not ready.'));continue
            goal=ExecuteCycle.Goal();goal.target_id=str(packet['target_id']);goal.quantity=int(packet['quantity'])
            active_cycle.update(pending=True,request_id=packet.get('request_id'),cancel=False)
            cycle.send_goal_async(goal).add_done_callback(cycle_response)
        with condition:
            state = dict(cache['state']) if cache['state'] else None
            fresh = cache['connected'] and time.monotonic()-cache['received_at'] <= .5
        teaching.publish()
        if state is None: return
        message = ConfiguredSupplyState()
        for key,value in state.items():
            if key in message.get_fields_and_field_types(): setattr(message,key,value)
        message.valid = message.valid and fresh
        if not fresh: message.diagnostic_detail = 'isaac_state_stale'
        if published_supply['epoch'] is not None and not message.valid:
            try:
                node.get_logger().warning('Configured supply became unavailable / '+str(message.diagnostic_detail))
            except Exception:
                pass
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        published_supply['epoch']=message.supply_epoch if message.valid else None
    node.create_timer(.1, publish)
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    send(dict(op='ready',ready=True))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        teaching.close()
        sock.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # The script path does not by itself expose the common scripts package.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try:
        serve(int(sys.argv[1]))
    except Exception as error:
        print(f'Configured Supply sidecar failed: {error}', file=sys.stderr)
        raise
