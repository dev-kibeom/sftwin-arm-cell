"""Headless equipment acceptance against an isolated PLC and the companion Mock."""
import argparse
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument('snapshot', type=Path)
parser.add_argument('exchange_dir', type=Path)
parser.add_argument('--equipment-port', type=int, required=True)
args = parser.parse_args()
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 480, 'height': 320, 'anti_aliasing': 0})
from isaac_runtime.runtime_workflow import prepare_startup_model, run_step
from isaac_runtime.runtime_configuration import load_runtime_configuration
from isaac_runtime import cell_supply

def commission():
    config = load_runtime_configuration(args.snapshot)
    config['cell']['conveyor']['connection']['port'] = args.equipment_port
    import os
    os.environ['ARM_CELL_RUNTIME_CONFIG'] = str(args.snapshot)
    prepare_startup_model({}, Path(os.environ['SFTWIN_PROJECT_ROOT']))
    namespace = dict(runtime_configuration=config)
    for step in ('setup_scene.py', 'snapshot_camera.py', 'build_actiongraph.py'):
        run_step(step, namespace)
    ctx = namespace['context']
    adapter = namespace['_material_supply_adapter']
    world = ctx['world']
    eq = ctx['equipment']
    spawned = []
    original_spawn, observe = eq.spawn, eq.observe
    delay_until = 0.
    def spawn(material, cycle):
        nonlocal delay_until
        assert cycle not in spawned
        receipt = original_spawn(material, cycle)
        assert receipt
        spawned.append(cycle)
        delay_until = time.monotonic()+1.5
        return receipt
    last_observed = None
    def delayed(permit):
        nonlocal last_observed
        sample = observe(permit)
        last_observed = sample
        if time.monotonic() < delay_until:
            sample.infeed = False
        return sample
    eq.spawn, eq.observe = spawn, delayed
    try:
        world.reset()
        run_step('bootstrap_tool.py', namespace)
        assert cell_supply.request_supply('ConveyorPart', 2)['accepted']
        assert cell_supply.set_automatic(True)['accepted']
        deadline = time.monotonic()+75
        paused = disconnected = False
        placed = set()
        last = None
        end_pause = reconnect = 0.
        while time.monotonic() < deadline:
            world.step(render=True)
            now = time.monotonic()
            if end_pause and now > end_pause:
                assert eq.speed == 0. and eq.cycle == 1
                cell_supply.set_automatic(True)
                end_pause = 0.
            if reconnect and now > reconnect:
                assert eq.speed == 0. and eq.cycle == 1
                (args.exchange_dir/'offline').unlink()
                reconnect = 0.
            if not adapter.packet:
                continue
            cmd = adapter.packet['command']
            state = cmd['cycle'], cmd['phase'], cmd['committed']
            if state != last:
                print('Equipment', state, 'receipt', adapter.feeder.last_generated,
                      'physical', last_observed, flush=True)
                last = state
            if state[:2] == (1, 21) and eq.speed and not paused:
                cell_supply.set_automatic(False)
                end_pause, paused = now+.7, True
            elif state[:2] == (1, 21) and eq.speed and paused and not end_pause and not disconnected:
                (args.exchange_dir/'offline').touch()
                reconnect, disconnected = now+1.6, True
            if cmd['cycle'] not in placed and (args.exchange_dir/f"place-{cmd['cycle']}").exists():
                sensors = observe(True)
                assert sensors.pick and sensors.extended and sensors.stopped, (cmd, sensors)
                handle = eq.body(eq.path)
                pose = eq.dc.get_rigid_body_pose(handle)
                pose.p = eq.layout['regions']['tray']['center']
                eq.dc.set_rigid_body_pose(handle, pose)
                eq.dc.wake_up_rigid_body(handle)
                placed.add(cmd['cycle'])
            if last_observed and last_observed.occupied:
                assert last_observed.path_clear and not last_observed.on_belt
            if state == (2, 0, 2):
                assert spawned == [1, 2] and placed == {1, 2} and eq.path is None
                assert paused and disconnected and not reconnect and not adapter.feeder.remaining
                print('PASS: two discharges, distinct reservations, same fixed pick, pause and link recovery', flush=True)
                return
        raise TimeoutError(f'last PLC state {last}, equipment {adapter.failure}, packet {adapter.packet}')
    finally:
        namespace['close_runtime']()
        world.stop()

try:
    commission()
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
