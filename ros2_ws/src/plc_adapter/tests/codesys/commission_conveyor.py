#!/usr/bin/env python3
"""Run against a fresh, isolated Conveyor PLC_PRG with no other equipment/robot writers."""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

from pymodbus.client.sync import ModbusTcpClient
import rclpy
from rclpy.executors import MultiThreadedExecutor

tests = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(tests))
sys.path.insert(0, str(tests.parents[3] / 'infra/isaac_sim/scripts'))
from test_modbus_action import RobotFixture, domain_id, process
from arm_cell_interfaces.action import ExecuteConfiguredGrasp
from conveyor_pnp_cell.equipment_io import ConveyorIO, Sensors
from conveyor_pnp_cell.supply import ConveyorFeeder, bind_confirmed_pick
from isaac_runtime import cell_supply


class ConveyorRobot(RobotFixture):
    def __init__(self):
        super().__init__()
        self.inventory.supply_id = 'conveyor_pick'
        self.inventory.material_id = 'ConveyorPart'
        self.inventory.slot_ids, self.inventory.occupied = [], []
        self.allow_reservation.set()

    def expose_slot(self, slot):
        if not self.inventory.reservation_id and slot not in self.inventory.slot_ids:
            self.inventory.slot_ids.append(slot)
            self.inventory.occupied.append(True)
        return self.inventory.slot_ids[-1:] == [slot] and self.inventory.occupied[-1]

    def execute(self, goal):
        slot = goal.request.slot_id
        self.goals.append(slot)
        assert self.finish.wait(30)
        result = ExecuteConfiguredGrasp.Result(material_placed=True)
        if slot == 'slot_3':
            result.result_code, result.failure_stage = 1, 'RETREAT'
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

        def start_adapter():
            adapter.enter_context(process('arm_cell_plc_adapter', 'plc_task_adapter',
                '--ros-args', '--params-file', str(profile / 'conveyor_pnp_codesys.yaml'),
                '--params-file', str(profile / 'conveyor_process.yaml'), '-p', f'port:={port}',
                '-p', f'receipt_file:={run / "receipt.sqlite"}'))

        start_adapter()
        feeder.bind_ui()
        stack.callback(cell_supply.clear_supply_port, feeder)
        assert cell_supply.request_supply('ConveyorPart', 3)['accepted']
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

        for cycle in range(1, 4):
            slot = f'slot_{cycle}'
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
            assert bind_confirmed_pick(cmd, cycle - 1, robot.expose_slot) == 0
            if cycle == 1:
                robot.safety.valid = False
                hold('no slot bound', lambda c: c.phase == 22 and not robot.goals, 1.)
            bound = bind_confirmed_pick(cmd, cycle, robot.expose_slot)
            assert bound == cycle
            if cycle == 1:
                hold('robot not ready', lambda c: c.phase == 22 and not robot.goals)
                robot.safety.valid = True
            until('one robot job', lambda c: c.phase == 30 and robot.goals[-1:] == [slot])
            sensors.pick, sensors.extended = False, False
            sensors.path_clear = True
            sensors.discharge_zone_clear = False
            if cycle == 2:
                adapter.close()
                sensors.occupied = True
                robot.finish.set()
                hold('unconfirmed result with occupied tray', lambda c: c.phase == 30
                     and c.committed < cycle and not c.feed and not c.discharge, 1.5)
                start_adapter()
            else:
                hold('robot busy', lambda c: c.phase == 30 and c.committed < cycle)
                robot.finish.set()
                until('terminal before occupancy', lambda c: c.phase == 40)
                hold('no tray observation', lambda c: c.committed < cycle and not c.discharge)
                valid = False
                sensors.occupied = True
                hold('invalid occupied observation', lambda c: c.phase == 40)
                valid = True
            cmd = until('commit', lambda c: c.phase == 50 and c.committed == cycle)
            assert (cmd.result_state, cmd.reason, cmd.placement) == (
                (6, 3, 2) if cycle == 3 else (4, 7, 2)), cmd
            hold('duplicate terminal during blocked discharge', lambda c: c.committed == cycle
                 and c.phase == 50 and not c.discharge and not c.feed)
            sensors.permitted = False
            sensors.discharge_zone_clear = True
            hold('discharge permission withheld', lambda c: c.phase == 50 and not c.discharge)
            if cycle == 1:
                robot.safety.valid = False
                hold('robot recovery', lambda c: c.phase == 50, 1.)
            sensors.permitted = True
            until('permitted discharge', lambda c: c.discharge)
            sensors.occupied = False
            if cycle == 1:
                hold('empty tray before Robot Ready', lambda c: c.phase == 51 and not c.feed)
                robot.safety.valid = True
            print('cycle', cycle, 'commit', cmd.committed, 'result', cmd.result_state,
                  cmd.reason, cmd.placement, 'goals', list(robot.goals), flush=True)
        until('stock exhausted', lambda c: c.phase == 0 and not c.feed)
        hold('idle', lambda c: c.cycle == c.committed == 3 and not c.feed)
        assert robot.goals == ['slot_1', 'slot_2', 'slot_3']
        assert spawned == [('ConveyorPart', n) for n in (1, 2, 3)] and not feeder.remaining


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, type=int)
    commission(parser.parse_args().port)
