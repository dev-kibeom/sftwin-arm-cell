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
    cache = {'state':None, 'scene':None, 'received_at':0., 'connected':True}
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
                        elif packet.get('op') == 'start_cycle':
                            operator_requests.append(packet)
                        elif packet.get('op') == 'state':
                            cache.update(state=packet['state'], scene=packet.get('scene'), received_at=time.monotonic())
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
    from tape_roll_cell.planning_scene import ConfiguredPlanningScene
    planning = ConfiguredPlanningScene(node)
    from rclpy.action import ActionClient
    from arm_cell_interfaces.action import ExecuteCycle
    cycle = ActionClient(node, ExecuteCycle, '/orchestration/execute_cycle')
    active_cycle = {'goal':None,'pending':False}
    def cycle_response(future):
        goal = future.result();active_cycle['pending']=False
        if goal is None or not goal.accepted:
            send(dict(op='mission_status',text='Mission 요청 거부: ROS/Safety/공급 준비를 확인하세요.'));return
        active_cycle['goal']=goal
        result = goal.get_result_async()
        def finished(future):
            outcome=future.result();active_cycle['goal']=None
            send(dict(op='mission_status',text='Mission 종료: '+outcome.result.diagnostic_detail))
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
    node.create_service(ConfiguredSupply, '/cell/configured_supply', service,
                        callback_group=ReentrantCallbackGroup())
    def publish():
        with condition:
            commands=list(operator_requests);operator_requests.clear()
        for packet in commands:
            if active_cycle['goal'] is not None or active_cycle['pending'] or not cycle.server_is_ready():
                send(dict(op='mission_status',text='Mission 실행 중이거나 ROS 준비 전입니다.'));continue
            goal=ExecuteCycle.Goal();goal.target_id=str(packet['target_id']);goal.quantity=int(packet['quantity'])
            active_cycle['pending']=True;cycle.send_goal_async(goal).add_done_callback(cycle_response)
        with condition:
            state = dict(cache['state']) if cache['state'] else None
            scene = cache['scene']
            fresh = cache['connected'] and time.monotonic()-cache['received_at'] <= .5
        if state is None: return
        message = ConfiguredSupplyState()
        for key,value in state.items():
            if key in message.get_fields_and_field_types(): setattr(message,key,value)
        message.valid = message.valid and fresh
        if not fresh: message.diagnostic_detail = 'isaac_state_stale'
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        if fresh and scene is not None: planning.publish(scene)
    node.create_timer(.1, publish)
    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    send(dict(op='ready',ready=True))
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
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
