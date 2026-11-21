#!/usr/bin/env python3
"""Run against a fresh, isolated Conveyor PLC_PRG with no other equipment/robot writers."""
import argparse
from contextlib import ExitStack
import os
import sqlite3
from pathlib import Path
import sys
import tempfile
import threading
import time

from pymodbus.client.sync import ModbusTcpClient
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

tests = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(tests))
sys.path.insert(0, str(tests.parents[3] / 'infra/isaac_sim/scripts'))
from test_modbus_action import RobotFixture, domain_id, process
from arm_cell_interfaces.action import ExecuteConfiguredGrasp
from arm_cell_interfaces.msg import MotionStatus
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply
from conveyor_pnp_cell.equipment_io import ConveyorIO, Sensors
from conveyor_pnp_cell.supply import ConveyorFeeder, bind_confirmed_pick
from isaac_runtime import cell_supply


class ConveyorRobot(RobotFixture):
    def __init__(self):
        super().__init__()
        self.busy_at = None
        self.admission_samples = []
        self.create_subscription(Bool, '/orchestration/mission_admission_available',
                                 lambda message: self.admission_samples.append(message.data), 10)
        self.pick_supply = ConveyorPickSupply('commission-epoch', supply_id='conveyor_pick',
            slot_id='cell/belt/pick', stream_id='test', material_id='ConveyorPart', material_type_ids=['ConveyorPart'])
        self.motion = MotionStatus(execution_state=MotionStatus.MOTION_STATE_IDLE)
        self.motion_pub = self.create_publisher(MotionStatus, '/motion/state', 10)

    def publish(self):
        self.inventory = self.pick_supply.state(valid=True)
        self.motion_pub.publish(self.motion)
        super().publish()

    def reserve(self, request, response):
        return self.pick_supply.handle(request, response, valid=True)

    def execute(self, goal):
        assert goal.request.slot_id == self.pick_supply.slot
        stream, instance = goal.request.material_instance_id.split('/')
        assert stream == 'test'
        if instance == '2' and self.busy_at is None:
            self.busy_at = time.monotonic()
            goal.abort()
            return ExecuteConfiguredGrasp.Result(result_code=10, failure_stage='configured_supply:execution_admission')
        if instance == '2':
            assert time.monotonic() - self.busy_at >= 1.
        self.goals.append(instance)
        assert self.finish.wait(30)
        result = ExecuteConfiguredGrasp.Result(material_placed=True)
        if instance == '6':
            goal.abort()
            return ExecuteConfiguredGrasp.Result(result_code=3, material_placed=False, failure_stage='TRANSFER')
        if instance in ('2', '3'):
            result.result_code = 3
            result.failure_stage = 'RETREAT' if instance == '2' else 'GO_HOME'
            goal.abort()
        else:
            goal.succeed()
        return result


def commission(port):
    profile = tests.parent / 'config'
    feeder = ConveyorFeeder('ConveyorPart', last_generated=0)
    sensors = Sensors(stopped=True, path_clear=True, drive_available=True,
                      pick_zone_clear=True, discharge_zone_clear=True, permitted=True)
    with ExitStack() as stack:
        client = ModbusTcpClient('127.0.0.1', port=port, timeout=.9)
        stack.callback(client.close)
        io = ConveyorIO(client)
        initial = io.commands()
        assert initial.cycle == initial.phase == initial.committed == 0, initial
        run = Path(stack.enter_context(tempfile.TemporaryDirectory()))
        os.environ.update(ROS_DOMAIN_ID=str(stack.enter_context(domain_id())),
                          ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(run / 'logs'))
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        robot = ConveyorRobot()
        stack.callback(robot.destroy_node)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(robot)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        stack.callback(thread.join)
        stack.callback(executor.shutdown)
        stack.callback(robot.finish.set)
        stack.enter_context(process('arm_cell_orchestration', 'configured_supply_mission.py',
            '--ros-args', '--params-file', str(profile / 'conveyor_process.yaml')))
        adapter = stack.enter_context(ExitStack())

        def start_adapter(target='ConveyorPart'):
            adapter.enter_context(process('arm_cell_plc_adapter', 'plc_task_adapter',
                '--ros-args', '--params-file', str(profile / 'conveyor_pnp_codesys.yaml'),
                '--params-file', str(profile / 'conveyor_process.yaml'), '-p', f'port:={port}',
                '-p', f'target_id:={target}', '-p', f'receipt_file:={run / "receipt.sqlite"}'))

        start_adapter(target='unregistered')  # Actual Mission admission rejection, no Task.
        feeder.bind_ui()
        stack.callback(cell_supply.clear_supply_port, feeder)
        assert cell_supply.request_supply('ConveyorPart', 5)['accepted']
        assert cell_supply.set_automatic(True)['accepted']
        bound = 0
        valid = True
        spawned = []

        def tick():
            io.publish(valid=valid, feeder=feeder, sensors=sensors, bound_cycle=bound)
            time.sleep(.04)
            return io.commands()

        def until(label, condition):
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                cmd = tick()
                if condition(cmd):
                    return cmd
            raise AssertionError(f'{label}: last scan {cmd}, goals {robot.goals}')

        def hold(label, condition, duration=.5):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                cmd = tick()
                assert condition(cmd), (label, cmd, robot.goals)

        def spawn(material, cycle):
            spawned.append((material, cycle))
            return True

        for cycle in range(1, 7):
            if cycle == 6:
                until('stock exhausted', lambda c: c.phase == 0 and not c.feed)
                hold('idle', lambda c: c.cycle == c.committed == 5 and not c.feed)
                assert cell_supply.set_automatic(False)['accepted']
                assert cell_supply.request_supply('ConveyorPart', 2)['accepted']
                assert cell_supply.set_automatic(True)['accepted']
            instance = str(cycle)
            robot.finish.clear()
            until('feed', lambda c: c.feed and c.cycle == cycle)
            feeder.feed(cycle, spawn)
            feeder.feed(cycle, spawn)
            hold('wait for infeed sensor', lambda c: c.phase == 10 and not c.run)
            sensors.infeed, sensors.path_clear = True, False
            until('conveyor run', lambda c: c.run)
            sensors.infeed, sensors.on_belt = False, True
            sensors.stopped = False
            if cycle == 1:
                time.sleep(1.2)  # Withhold all equipment samples past the PLC watchdog.
                stale = io.commands()
                assert stale.phase == 21 and not stale.run and not stale.feed
                until('fresh same transfer', lambda c: c.run and c.cycle == cycle)
                sensors.drive_available = False
                until('drive paused', lambda c: c.phase == 21 and not c.run)
                hold('part remains on belt', lambda c: c.phase == 21 and not c.run and not c.feed)
                sensors.drive_available = True
                until('same transfer resumed', lambda c: c.run and c.cycle == cycle)
            hold('pick sensor delayed', lambda c: c.run and not c.confirmed)
            sensors.pick, sensors.on_belt = True, False
            until('stop requested', lambda c: c.phase == 22 and not c.run)
            hold('belt decelerating', lambda c: not c.extend and not c.confirmed)
            sensors.stopped = True
            until('stopper command', lambda c: c.extend)
            sensors.extended = True
            cmd = until('pick confirmed', lambda c: c.confirmed)
            assert bind_confirmed_pick(cmd, cycle - 1, feeder.material_type_id, robot.pick_supply.expose_arrival) == 0
            if cycle == 1:
                robot.safety.valid = False
                hold('no slot bound', lambda c: c.phase == 22 and not robot.goals, 1.)
            bound = bind_confirmed_pick(cmd, cycle, feeder.material_type_id, robot.pick_supply.expose_arrival)
            assert bound == cycle
            if cycle == 1:
                hold('robot not ready', lambda c: c.phase == 22 and not robot.goals)
                robot.safety.valid = True
                until('request issued', lambda c: c.phase in (23, 30))
                def rejection_is(command_id):
                    # Request ID, phase and terminal state must come from the same PLC scan.
                    words = client.read_input_registers(0, 27, unit=1).registers
                    return (words[0] << 16 | words[1], words[18], words[26]) == (command_id, 23, 7)

                until('explicit rejection', lambda c: rejection_is(1))

                def awaiting_rearm(c):
                    return c.confirmed and not robot.goals and rejection_is(rejected_id)

                rejected_id = 1
                sample_start = len(robot.admission_samples)
                hold('bound Pick and unchanged TRUE across admission samples', awaiting_rearm, 1.5)
                samples = robot.admission_samples[sample_start:]
                assert len(samples) >= 3 and all(samples), samples
                robot.safety.valid = False
                hold('new FALSE admission sample', awaiting_rearm, .8)
                robot.safety.valid = True
                until('one new rejection after FALSE to TRUE', lambda c: rejection_is(2))
                rejected_id = 2
                hold('each recovery allows only one retry', awaiting_rearm, 1.)
                adapter.close()
                start_adapter()
                hold('configuration repaired, admission remains TRUE', awaiting_rearm, 1.5)
                assert cell_supply.set_automatic(False)['accepted']
                hold('operator disabled automatic after repair', lambda c: rejection_is(2))
                assert cell_supply.set_automatic(True)['accepted']
            if cycle == 2:
                until('pre-BT executor busy', lambda c: c.phase == 23 and c.reason == 9)
                hold('busy retry delay', lambda c: c.phase == 23 and c.confirmed and instance not in robot.goals, .6)
            until('one robot job', lambda c: c.phase == 30 and robot.goals[-1:] == [instance])
            if cycle == 6:
                robot.finish.set()
                until('executed failure without placement', lambda c: c.phase == 39)
                hold('same part still present, no replay', lambda c: c.phase == 39 and c.committed == 5
                     and (c.result_state, c.reason, c.placement) == (6, 3, 1) and not c.feed and not c.discharge)
                break
            sensors.pick, sensors.extended = False, False
            sensors.path_clear = True
            sensors.discharge_zone_clear = False
            if cycle == 2:
                adapter.close()
                robot.motion.execution_state = MotionStatus.MOTION_STATE_STOPPING
                sensors.occupied = True
                robot.finish.set()
                hold('unconfirmed result with occupied tray', lambda c: c.phase == 30
                     and c.committed < cycle and not c.feed and not c.discharge, 1.5)
                start_adapter()
            else:
                if cycle == 3:
                    robot.safety.motion_capability.value = 1
                hold('robot busy', lambda c: c.phase == 30 and c.committed < cycle)
                robot.finish.set()
                until('terminal before occupancy', lambda c: c.phase == 40)
                hold('no tray observation', lambda c: c.committed < cycle and not c.discharge)
                valid = False
                sensors.occupied = True
                hold('invalid occupied observation', lambda c: c.phase == 40)
                valid = True
            cmd = until('commit', lambda c: c.phase == 50 and c.committed == cycle)
            expected_result = {2: (6, 3, 2), 3: (6, 4, 2)}.get(cycle, (4, 7, 2))
            assert (cmd.result_state, cmd.reason, cmd.placement) == expected_result, cmd
            assert not robot.pick_supply.expose_arrival(cycle, feeder.material_type_id), 'placed arrival became reservable again'
            hold('duplicate terminal during blocked discharge', lambda c: c.committed == cycle
                 and c.phase == 50 and not c.discharge and not c.feed)
            sensors.permitted = False
            sensors.discharge_zone_clear = True
            hold('discharge permission withheld', lambda c: c.phase == 50 and not c.discharge)
            if cycle == 1:
                robot.safety.valid = False
                hold('robot recovery', lambda c: c.phase == 50, 1.)
            if cycle == 3:
                robot.safety.motion_capability.value = 2
            sensors.permitted = True
            until('permitted discharge', lambda c: c.discharge)
            sensors.occupied = False
            if cycle == 1:
                hold('empty tray before Mission admission', lambda c: c.phase == 51 and not c.feed)
                robot.safety.valid = True
            if cycle == 2:
                hold('stop incomplete after discharge', lambda c: c.phase == 51 and not c.feed)
                robot.motion.execution_state = MotionStatus.MOTION_STATE_STOPPED
                robot.motion.backend_inactivity_confirmed = True
            print('cycle', cycle, 'commit', cmd.committed, 'result', cmd.result_state,
                  cmd.reason, cmd.placement, 'goals', list(robot.goals), flush=True)
        assert robot.goals == [str(n) for n in range(1, 7)]
        adapter.close()
        with sqlite3.connect(run / 'receipt.sqlite') as db:
            history = db.execute('SELECT command_id,state,reason,placement FROM receipt ORDER BY command_id').fetchall()
        assert [row[0] for row in history] == list(range(len(history)))
        assert [row[0] for row in history if row[1] == 7] == [1, 2]
        assert [row[1:] for row in history if row[1] not in (0, 7)] == [
            (4, 7, 2), (6, 9, 1), (6, 3, 2), (6, 4, 2), (4, 7, 2), (4, 7, 2), (6, 3, 1)]
        print('Adapter receipts', history, flush=True)
        assert spawned == [('ConveyorPart', n) for n in range(1, 7)] and feeder.remaining == 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, type=int)
    commission(parser.parse_args().port)
