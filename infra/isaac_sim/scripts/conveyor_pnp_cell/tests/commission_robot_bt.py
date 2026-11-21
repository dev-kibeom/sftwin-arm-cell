"""Observe three consecutive real Robot BT cycles on a fresh Conveyor PLC stream."""
import os
from pathlib import Path
import sys
import time

ROOT = Path(os.environ['SFTWIN_PROJECT_ROOT'])
sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))


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
        first_batch = 2 if len(types) > 1 else 3
        assert cell_supply.request_supply(types[0], first_batch)['accepted']
        assert cell_supply.set_automatic(True)['accepted']
        tool = namespace['grasp_manager']
        reservations = {}
        released, occupied, discharged = set(), set(), set()
        previous = None
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            start = time.monotonic()
            world.step(render=True)
            time.sleep(max(0, 1/60 - (time.monotonic() - start)))
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
            state = (cycle, cmd['phase'], cmd['committed'], holding, sensors.occupied)
            if state != previous:
                print('CYCLE', state, device, flush=True)
                previous = state
            if cmd['phase'] == 39:
                raise RuntimeError(f'PLC requires reconciliation: {cmd}')
            if holding == 'held':
                assert lease and lease['slot_id'] == 'conveyor_pnp_cell/CV-01/pick'
                assert lease['material_instance_id'] == f'{equipment.stream_id}/{cycle}'
                assert lease['material_type_id'] == types[int(cycle > first_batch)]
                target = equipment.material_path(lease['material_type_id'], lease['material_instance_id'])
                assert target and device['requested_body'] == target
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
                    if cycle == first_batch and cycle < 3:
                        assert cell_supply.set_automatic(False)['accepted']
                        assert cell_supply.request_supply(types[1], 1)['accepted']
                        assert cell_supply.set_automatic(True)['accepted']
                    print('DISCHARGED', cycle, cmd, 'joints_rad',
                          context['robot'].get_joint_positions().tolist(), flush=True)
            if discharged == {1, 2, 3} and cmd['phase'] == 0:
                assert len({epoch for epoch, _ in reservations.values()}) == 1
                assert adapter.feeder.remaining == 0
                print('PASS three Robot BT cycles', flush=True)
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
