from pathlib import Path
import sys


SCRIPT_ROOT = Path(__file__).parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from pnp_validation.graph_view_recovery import delete_with_graph_rebuild


class _Prim:
    def __init__(self, valid):
        self.valid = valid

    def IsValid(self):
        return self.valid


class _Stage:
    def __init__(self, events):
        self.events = events
        self.graph_present = True

    def GetPrimAtPath(self, path):
        self.events.append(("lookup", path))
        return _Prim(self.graph_present)

    def RemovePrim(self, path):
        self.events.append(("remove_graph", path))
        self.graph_present = False


def test_fixture_delete_releases_controller_node_before_stage_mutation_and_rebuilds():
    events = []
    stage = _Stage(events)

    deleted = delete_with_graph_rebuild(
        stage=stage,
        graph_path="/World/Cell/ROS2/ActionGraph",
        release_graph_view=lambda: events.append(("release_graph_view",)),
        delete_fixture=lambda: events.append(("delete_fixture",)),
        rebuild_graph=lambda: events.append(("rebuild_graph",)),
    )

    assert deleted is None
    assert events == [
        ("lookup", "/World/Cell/ROS2/ActionGraph"),
        ("release_graph_view",),
        ("delete_fixture",),
        ("rebuild_graph",),
    ]


def test_fixture_delete_rebuilds_graph_when_delete_fails():
    events = []

    def fail_delete():
        events.append(("delete_fixture",))
        raise RuntimeError("delete failed")

    try:
        delete_with_graph_rebuild(
            stage=_Stage(events),
            graph_path="/World/Cell/ROS2/ActionGraph",
            release_graph_view=lambda: events.append(("release_graph_view",)),
            delete_fixture=fail_delete,
            rebuild_graph=lambda: events.append(("rebuild_graph",)),
        )
    except RuntimeError as error:
        assert str(error) == "delete failed"
    else:
        raise AssertionError("fixture deletion error was swallowed")

    assert events[-1] == ("rebuild_graph",)
