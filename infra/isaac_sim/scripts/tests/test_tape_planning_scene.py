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
    packet=dict(frame_id='world',rolls=[],finger_links=[],holding_state='unknown',selected_slot_id='',static_boxes=[])
    planning.publish(packet)
    assert not messages[-1].robot_state.attached_collision_objects
    packet['static_boxes']=[dict(id='pallet',position=[0.,0.,0.],dimensions=[1.,1.,.02])]
    planning.publish(packet)
    packet['static_boxes']=[]
    planning.publish(packet)
    removed=messages[-1].world.collision_objects
    assert len(removed)==1 and removed[0].id=='pallet' and removed[0].operation==removed[0].REMOVE
