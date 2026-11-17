"""Empty/changed Supply scenes must not crash or leave ghost collision objects."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
pytest.importorskip('moveit_msgs.msg')
from tape_roll_cell.planning_scene import ConfiguredPlanningScene


def test_empty_supply_and_pallet_removal_are_valid_diffs():
    messages=[]
    node=SimpleNamespace(create_publisher=lambda *args:SimpleNamespace(publish=messages.append),
                         create_client=lambda *args:SimpleNamespace(service_is_ready=lambda:False))
    planning=ConfiguredPlanningScene(node)
    from moveit_msgs.msg import AllowedCollisionMatrix
    planning.acm=AllowedCollisionMatrix()
    packet=dict(frame_id='world',rolls=[],finger_links=[],holding_state='unknown',selected_slot_id='',static_boxes=[])
    planning.publish(packet)
    assert not messages[-1].robot_state.attached_collision_objects
    packet['static_boxes']=[dict(id='pallet',position=[0.,0.,0.],dimensions=[1.,1.,.02])]
    planning.publish(packet)
    packet['static_boxes']=[]
    planning.publish(packet)
    removed=messages[-1].world.collision_objects
    assert len(removed)==1 and removed[0].id=='pallet' and removed[0].operation==removed[0].REMOVE


def test_held_roll_is_removed_once_and_released_roll_returns_to_world():
    import numpy as np
    from moveit_msgs.msg import AllowedCollisionMatrix
    messages=[]
    node=SimpleNamespace(create_publisher=lambda *args:SimpleNamespace(publish=messages.append),
                         create_client=lambda *args:SimpleNamespace(service_is_ready=lambda:False))
    planning=ConfiguredPlanningScene(node)
    planning.acm=AllowedCollisionMatrix()
    row=dict(id='roll',slot_id='slot_0',position=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.])
    packet=dict(frame_id='base_link',rolls=[row],finger_links=[],holding_state='unknown',
                selected_slot_id='slot_0',material=dict(inner_radius_m=.06,outer_radius_m=.07,width_m=.017))
    planning.publish(packet)
    packet.update(holding_state='held',accepted_relation=np.eye(4).reshape(-1).tolist())
    for _ in range(100):planning.publish(packet)
    removes=[obj for msg in messages for obj in msg.world.collision_objects if obj.operation==obj.REMOVE]
    assert len(removes)==1
    attaches=[obj for msg in messages for obj in msg.robot_state.attached_collision_objects if obj.object.operation==obj.object.ADD]
    assert len(attaches)==1
    packet['holding_state']='released'
    planning.publish(packet)
    assert any(obj.id=='roll' and obj.operation==obj.ADD for obj in messages[-1].world.collision_objects)
    assert messages[-1].robot_state.attached_collision_objects[0].object.operation==messages[-1].robot_state.attached_collision_objects[0].object.REMOVE


def test_producer_waits_until_moveit_acknowledges_world_frame():
    from concurrent.futures import Future
    from moveit_msgs.msg import PlanningScene
    from geometry_msgs.msg import TransformStamped
    messages=[];queries=[]
    def query(request):
        future=Future();queries.append(future);return future
    node=SimpleNamespace(create_publisher=lambda *args:SimpleNamespace(publish=messages.append),
        create_client=lambda *args:SimpleNamespace(service_is_ready=lambda:True,call_async=query))
    planning=ConfiguredPlanningScene(node)
    packet=dict(frame_id='world',rolls=[],finger_links=[],holding_state='unknown',selected_slot_id='')
    planning.publish(packet)
    scene=PlanningScene();queries[-1].set_result(SimpleNamespace(scene=scene))
    planning.publish(packet)
    assert not messages
    transform=TransformStamped();transform.child_frame_id='world'
    transform.header.frame_id='base_link';transform.transform.rotation.w=1.
    transform.transform.translation.z=-.5
    scene.fixed_frame_transforms=[transform]
    packet['static_boxes']=[dict(id='fixture',position=[0.,0.,.8],dimensions=[1.,1.,.02])]
    queries[-1].set_result(SimpleNamespace(scene=scene))
    planning.publish(packet)
    assert len(messages)==1
    obj=messages[0].world.collision_objects[0]
    assert obj.header.frame_id=='base_link'
    assert obj.pose.position.z==pytest.approx(.3)


def test_moveit_model_frame_needs_no_external_transform():
    from concurrent.futures import Future
    from moveit_msgs.msg import PlanningScene
    messages=[];future=Future()
    node=SimpleNamespace(create_publisher=lambda *args:SimpleNamespace(publish=messages.append),
        create_client=lambda *args:SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:future))
    planning=ConfiguredPlanningScene(node)
    packet=dict(frame_id='world',rolls=[],finger_links=[],holding_state='unknown',selected_slot_id='')
    planning.publish(packet)
    scene=PlanningScene();scene.robot_state.multi_dof_joint_state.header.frame_id='world'
    future.set_result(SimpleNamespace(scene=scene))
    planning.publish(packet)
    assert len(messages)==1
