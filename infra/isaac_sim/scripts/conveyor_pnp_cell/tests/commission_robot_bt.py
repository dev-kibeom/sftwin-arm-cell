"""Observe five consecutive real Robot BT cycles on a fresh Conveyor PLC stream."""
import os
import signal
import sqlite3
from contextlib import closing
from pathlib import Path
import sys
import time

ROOT = Path(os.environ['SFTWIN_PROJECT_ROOT'])
sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))


def stream_receipts(config, stream):
    history = Path(config['history_db_path']).expanduser().resolve().as_uri() + '?mode=ro'
    with closing(sqlite3.connect(history, uri=True)) as supply:
        source = supply.execute(
            'SELECT receipt_file FROM receipt_sources WHERE stream_id=?', (stream,)).fetchone()
    if not source:
        raise AssertionError(f'No receipt source for equipment stream {stream}')
    return Path(source[0]).expanduser().resolve()


def task_identity(database, lease):
    with closing(sqlite3.connect(Path(database).as_uri() + '?mode=ro', uri=True)) as connection:
        return connection.execute(
            """SELECT command_id, hex(goal_uuid), task_goal_uuid FROM receipt
               JOIN configured_task USING(command_id) WHERE reservation_id=? AND material_instance_id=?
               AND task_goal_uuid IS NOT NULL""",
            (lease['reservation_id'], lease['material_instance_id'])).fetchone()


def verify_production_records(config, stream, reservations):
    history = Path(config['history_db_path']).expanduser().resolve().as_uri() + '?mode=ro'
    receipts = stream_receipts(config, stream).as_uri() + '?mode=ro'
    with closing(sqlite3.connect(history, uri=True)) as supply, closing(sqlite3.connect(receipts, uri=True)) as robot:
        executions = []
        for cycle, (epoch, token) in reservations.items():
            instance = f'{stream}/{cycle}'
            assert supply.execute(
                'SELECT supply_epoch,reservation_id,status FROM reservations WHERE instance_id=?',
                (instance,)).fetchall() == [(epoch, token, 'RELEASED')]
            assert supply.execute(
                'SELECT committed,placement FROM plc_cycles WHERE stream_id=? AND plc_number=?',
                (stream, cycle)).fetchone() == (1, 2)
            attempt, = robot.execute(
                """SELECT goal_uuid,task_goal_uuid,state,placement,reservation_id FROM receipt
                   JOIN configured_task USING(command_id) WHERE material_instance_id=?""",
                (instance,)).fetchall()
            assert attempt[0] and attempt[1] and attempt[2:] == (4, 2, token)
            executions.append(attempt[:2])
        assert len({mission for mission, _ in executions}) == len(executions)
        assert len({task for _, task in executions}) == len(executions)


def commission():
    from isaac_runtime import cell_supply
    from isaac_runtime.runtime_workflow import prepare_startup_model, run_step

    namespace = {}
    prepare_startup_model(namespace, ROOT)
    for step in ('setup_scene.py', 'build_actiongraph.py'):
        run_step(step, namespace)
    context = namespace['context']
    world, equipment = context['world'], context['equipment']
    adapter = namespace['_material_supply_adapter']
    try:
        world.reset()
        run_step('bootstrap_tool.py', namespace)
        types = adapter.feeder.material_type_ids
        cycle_count = 5
        first_batch = 2 if len(types) > 1 else cycle_count
        assert cell_supply.set_automatic(True)['accepted']
        assert cell_supply.request_supply(types[0], first_batch)['accepted']
        tool = namespace['grasp_manager']
        reservations = {}
        released, occupied, discharged = set(), set(), set()
        previous = None
        restart = '--restart-at-pick' in sys.argv
        original = original_task = old_process = None
        resumed = False
        killed = False
        deadline = time.monotonic() + 60 * cycle_count
        while time.monotonic() < deadline:
            start = time.monotonic()
            world.step(render=True)
            time.sleep(max(0, 1/60 - (time.monotonic() - start)))
            if old_process is not None and not killed and time.monotonic() >= kill_at:
                old_process.kill()
                killed = True
            if not adapter.packet:
                continue
            cmd = adapter.packet['command']
            cycle = cmd['cycle']
            if not cycle:
                continue
            try:
                sensors = equipment.observe(adapter.packet['permitted'])
            except ValueError:
                # A new PhysX body may not have an observation on its creation frame.
                continue
            holding = tool.holding_observation().state.value
            device = tool.gripper.observation()
            lease = adapter.packet.get('reservation')
            if restart and cycle == 2 and cmd['phase'] == 30 and sensors.pick and lease and original is None:
                original_task = task_identity(stream_receipts(adapter.sidecar_config, equipment.stream_id), lease)
                if original_task:
                    assert equipment.material_path(lease['material_type_id'], lease['material_instance_id'])
                    original = dict(lease)
                    old_process = adapter.sidecar.process
                    old_process.send_signal(signal.SIGSTOP)
                    kill_at = time.monotonic() + 5.5
                    print('SUPPLY_INTERRUPTED', old_process.pid, original, original_task, flush=True)
            if killed and not resumed and lease and adapter.sidecar.process.pid != old_process.pid:
                assert lease == original and old_process.poll() is not None
                assert task_identity(stream_receipts(adapter.sidecar_config, equipment.stream_id), lease) == original_task
                resumed = True
                print('SUPPLY_RECONNECTED', adapter.sidecar.process.pid, lease, original_task, flush=True)
            state = (cycle, cmd['phase'], cmd['committed'], holding, sensors.occupied)
            if state != previous:
                print('CYCLE', state, device, flush=True)
                previous = state
            if cmd['phase'] == 39:
                raise RuntimeError(f'PLC requires reconciliation: {cmd}')
            if holding == 'held':
                if cycle == 2 and original and not resumed and not lease:
                    lease = original
                assert lease and lease['slot_id'] == 'conveyor_pnp_cell/CV-01/pick'
                assert lease['material_instance_id'] == f'{equipment.stream_id}/{cycle}'
                assert lease['material_type_id'] == types[int(cycle > first_batch)]
                target = equipment.material_path(lease['material_type_id'], lease['material_instance_id'])
                assert target
                assert device['gripped_objects'] == [target]
                identity = (lease['supply_epoch'], lease['reservation_id'])
                if cycle not in reservations:
                    assert cycle == len(reservations) + 1 and len(discharged) == cycle - 1
                    assert identity not in reservations.values()
                    print('HELD', cycle, target, lease['material_type_id'], lease['material_instance_id'],
                          identity, flush=True)
                    reservations[cycle] = identity
                assert reservations[cycle] == identity
            if cycle in reservations and holding == 'released':
                assert not device['gripped_objects']
                released.add(cycle)
                if sensors.occupied:
                    occupied.add(cycle)
            if cmd['committed'] == cycle:
                assert cycle in released and cycle in occupied
                assert (cmd['result_state'], cmd['placement']) == (4, 2), cmd
                if equipment.path is None and cycle not in discharged:
                    assert sensors.discharge_zone_clear and not sensors.occupied
                    assert not lease and not device['gripped_objects']
                    discharged.add(cycle)
                    if cycle == first_batch and cycle < cycle_count:
                        assert cell_supply.request_supply(types[1], cycle_count - first_batch)['accepted']
                    print('DISCHARGED', cycle, cmd, 'joints_rad',
                          context['robot'].get_joint_positions().tolist(), flush=True)
            if len(discharged) == cycle_count and cmd['phase'] == 0:
                assert len({epoch for epoch, _ in reservations.values()}) == 1
                assert adapter.feeder.remaining == 0
                if restart:
                    assert resumed and original_task == task_identity(stream_receipts(adapter.sidecar_config, equipment.stream_id), original)
                    print('PASS Sidecar restart and next production', flush=True)
                verify_production_records(adapter.sidecar_config, equipment.stream_id, reservations)
                print(f'PASS {cycle_count} Robot BT cycles', flush=True)
                return
        raise TimeoutError(f'Last sample {previous}, equipment diagnostic {adapter.failure}')
    finally:
        namespace['close_runtime']()
        world.stop()


if __name__ == '__main__':
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, width=320, height=240, anti_aliasing=0))
    try:
        commission()
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()
