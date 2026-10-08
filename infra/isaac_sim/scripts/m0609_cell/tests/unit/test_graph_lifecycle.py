from pathlib import Path
import sys


SCRIPT_ROOT = Path(__file__).parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from graph_builder.graph_lifecycle import release_graph_controller_node


def test_release_articulation_controller_uses_omnigraph_node_lifecycle():
    calls = []

    class GraphController:
        @staticmethod
        def delete_node(**kwargs):
            calls.append(kwargs)

    release_graph_controller_node(GraphController, "/World/Cell/ROS2/ActionGraph")

    assert calls == [
        {
            "node_id": "/World/Cell/ROS2/ActionGraph/ArticulationController",
            "ignore_if_missing": True,
            "undoable": False,
        }
    ]
