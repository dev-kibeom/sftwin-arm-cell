"""Live M0609/Robotiq runtime smoke; run with Isaac's python.sh.

Export Runtime Configuration first. --pre-build explicitly regenerates the
selected Assembly; Startup itself never performs Pre Build.
"""
import os
import sys
import runpy
import traceback
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[6]
os.environ['SFTWIN_PROJECT_ROOT'] = str(ROOT)
os.environ.setdefault('ARM_CELL_RUNTIME_CONFIG', str(ROOT / '.run_records/runtime_configuration.json'))
os.environ['SFTWIN_HEADLESS'] = '1'
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
try:
    sys.path.insert(0, str(ROOT / "infra/isaac_sim/scripts"))
    if '--pre-build' in sys.argv:
        runpy.run_path(str(ROOT / 'infra/isaac_sim/scripts/0_pre_build.py'), run_name='__main__')
    source = str(ROOT / "infra/isaac_sim/scripts/isaac_arm_cell_startup.py")
    ns = runpy.run_path(source)
    task = ns['_ARM_CELL_STARTUP_TASK']
    for _ in range(100):
        app.update()
        if task.done(): break
    assert task.done(), 'Startup not completed within smoke observation window'
    task.result()
    from isaac_runtime import runtime_workflow
    runtime = runtime_workflow.startup_namespace
    session = runtime['runtime_session']
    stage = session.gripper.stage
    configuration = runtime['runtime_configuration']
    from pxr import UsdPhysics
    assert stage.GetPrimAtPath(configuration['cell']['camera']['prim_path']).IsValid()
    joints = {prim.GetName(): prim for prim in stage.Traverse()
              if str(prim.GetPath()).startswith(runtime['_robot_binding'].robot_path + '/')
              and prim.GetName() in runtime['_robot_binding'].joint_names}
    stiffness = {name: UsdPhysics.DriveAPI.Get(prim, 'angular').GetStiffnessAttr().Get()
                 for name, prim in joints.items()}
    assert session.physics_sub is not None
    assert runtime['_material_supply_adapter'] is not None
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    adapter = runtime["_material_supply_adapter"]
    print("SUPPLY_SIDECAR_AVAILABLE", adapter.sidecar.available, flush=True)
    dc = session.gripper.dc
    arm_joint = dc.find_articulation_dof(session.gripper.articulation, 'joint_1')
    state_pos = session.gripper._dynamic_control.STATE_POS
    initial_joint = dc.get_dof_state(arm_joint, state_pos).pos
    dc.set_dof_position_target(arm_joint, initial_joint + 0.08)
    for _ in range(60): app.update()
    moved_joint = dc.get_dof_state(arm_joint, state_pos).pos
    print('ROBOT_JOINT_RAD', initial_joint, moved_joint, flush=True)
    assert abs(moved_joint - initial_joint) > 0.03
    assert adapter.load_demonstration('single') == 1
    demo_paths = tuple(adapter._demonstration_paths)
    assert demo_paths
    session.gripper.close()
    for _ in range(30): app.update()
    closed_width = session.gripper.get_width()
    session.gripper.open()
    for _ in range(30): app.update()
    open_width = session.gripper.get_width()
    print("TOOL_WIDTHS_MM", closed_width, open_width, flush=True)
    assert open_width > closed_width + 20
    first = runtime['_live_test_session'].session_id
    timeline.pause()
    app.update()
    timeline.play()
    app.update()
    assert runtime['_live_test_session'].session_id == first
    timeline.stop()
    for _ in range(5): app.update()
    assert runtime['_live_test_session'] is None
    for name, target in zip(runtime['_robot_binding'].joint_names,
                            configuration['home_joint_positions']):
        drive = UsdPhysics.DriveAPI.Get(joints[name], 'angular')
        assert math.isclose(drive.GetTargetPositionAttr().Get(), math.degrees(target), abs_tol=1e-5)
        assert drive.GetStiffnessAttr().Get() == stiffness[name]
    assert all(not adapter.stage.GetPrimAtPath(path).IsValid() for path in demo_paths)
    timeline.play()
    for _ in range(5): app.update()
    assert runtime['_live_test_session'].session_id != first
    runtime_workflow.run_role('after_play_headless', runtime)
    assert runtime['runtime_session'] is not session
    assert session.physics_sub is None
    timeline.stop()
    app.update()
    runtime['close_runtime']()
    print('RUNTIME_SMOKE_PASS', flush=True)
except BaseException:
    traceback.print_exc()
    print('RUNTIME_SMOKE_FAIL', flush=True)
    raise
finally:
    app.close()
