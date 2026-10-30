import ast
import json
import socket
import sys
import types
from pathlib import Path

from scene_builder.material_supply_sidecar import (
    MaterialSupplySidecarClient,
    completion_payload,
)
from scene_builder.material_supply import MaterialSupplyProfile, sample_initial_poses
from scene_builder.material_supply_adapter import IsaacMaterialSupplyAdapter
from scene_builder.material_supply_run import MaterialSupplyRun
from material_supply_ros_sidecar import (
    _accept_completion_once,
    _transfer_completion_message,
    trigger_payload,
)


REPOSITORY = Path(__file__).resolve().parents[6]


def test_kit_material_supply_path_has_no_rclpy_import():
    paths = (
        REPOSITORY / "infra/isaac_sim/scripts/m0609_cell/runtime_steps/setup_scene.py",
        REPOSITORY
        / "infra/isaac_sim/scripts/m0609_cell/scene_builder/material_supply_adapter.py",
        REPOSITORY
        / "infra/isaac_sim/scripts/m0609_cell/scene_builder/material_supply_sidecar.py",
    )
    for path in paths:
        tree = ast.parse(path.read_text())
        imports = [
            node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        ]
        imports.extend(
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        )
        assert not any(name == "rclpy" or name.startswith("rclpy.") for name in imports)


def test_ros_boundary_lives_in_python310_sidecar_and_carries_both_directions():
    child = (
        REPOSITORY / "infra/isaac_sim/scripts/m0609_cell/material_supply_ros_sidecar.py"
    ).read_text()
    client = (
        REPOSITORY
        / "infra/isaac_sim/scripts/m0609_cell/scene_builder/material_supply_sidecar.py"
    ).read_text()

    assert "import rclpy" in child
    assert '"/integration/accepted_handoff"' in child
    assert '"/material_supply/transfer_completion"' in child
    assert '"/usr/bin/python3.10"' in client
    assert "source /opt/ros/humble/setup.bash" in client


class _FakeSidecar:
    available = True

    def __init__(self):
        self.sent = []
        self.closed = False

    def poll_messages(self):
        return []

    def flush_outgoing(self):
        pass

    def send_completion(self, result):
        self.sent.append(result)

    def close(self):
        self.closed = True


class _FakePrim:
    def __init__(self, valid):
        self.valid = valid

    def IsValid(self):
        return self.valid


class _FakeStage:
    def __init__(self):
        self.paths = set()
        self.queried_paths = []

    def GetPrimAtPath(self, path):
        self.queried_paths.append(path)
        return _FakePrim(path in self.paths)

    def RemovePrim(self, path):
        self.paths = {
            candidate
            for candidate in self.paths
            if candidate != path and not candidate.startswith(path.rstrip("/") + "/")
        }

    def Traverse(self):
        return []


class _ObservedStage(_FakeStage):
    def __init__(self):
        super().__init__()
        self.positions = {}

    def GetPrimAtPath(self, path):
        self.queried_paths.append(path)
        return types.SimpleNamespace(IsValid=lambda: path in self.paths, path=path)


def _install_fake_usd(monkeypatch, stage):
    class _Matrix:
        def __init__(self, point):
            self.point = point

        def ExtractTranslation(self):
            return self.point

    class _Xformable:
        def __init__(self, prim):
            self.prim = prim

        def ComputeLocalToWorldTransform(self, _time_code):
            return _Matrix(stage.positions[self.prim.path])

    monkeypatch.setitem(
        sys.modules,
        "pxr",
        types.SimpleNamespace(
            Usd=types.SimpleNamespace(
                TimeCode=types.SimpleNamespace(Default=lambda: None)
            ),
            UsdGeom=types.SimpleNamespace(Xformable=_Xformable),
        ),
    )


def _adapter(sidecar, stage=None, timeline=None):
    profile = MaterialSupplyProfile(
        material_type="RawPart",
        shape="box",
        dimensions_m=(0.04, 0.04, 0.04),
        mass_kg=0.1,
        receiving_x_m=(-0.1, 0.1),
        receiving_y_m=(-0.1, 0.1),
        spawn_height_above_shelf_m=0.1,
        spawn_interval_s=0.01,
        orientation_rpy_rad=((-1, 1), (-1, 1), (-1, 1)),
    )
    return IsaacMaterialSupplyAdapter(
        stage=stage or _FakeStage(),
        sidecar=sidecar,
        profile=profile,
        shelf_origin_m=(0.0, 0.0, 0.0),
        delivery_deadline_ms=1000,
        timeline=timeline,
    )


def test_usd_delivery_prim_path_prefixes_numeric_delivery_identity():
    delivery_id = "88fd5c9541a946a0a1b2e9d69717de92"
    stage = _FakeStage()
    adapter = _adapter(_FakeSidecar(), stage)

    adapter._delivery_prim_exists(delivery_id)

    assert stage.queried_paths == [
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_"
        "88fd5c9541a946a0a1b2e9d69717de92"
    ]


def test_stop_cleanup_keeps_failed_prim_path_for_retry(caplog):
    class _FailingStage(_FakeStage):
        def RemovePrim(self, path):
            raise RuntimeError("stage is not writable")

    stage = _FailingStage()
    adapter = _adapter(_FakeSidecar(), stage)
    delivery_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_" + "01" * 16
    )
    adapter._delivery_paths.add(delivery_path)
    adapter._deliveries.add("01" * 16)
    adapter._pending_parts.append(("01" * 16, 0, None, 1))

    adapter.reset_generated_materials()

    assert adapter._delivery_paths == {delivery_path}
    assert adapter._deliveries == {"01" * 16}
    assert not adapter._pending_parts
    captured_log = caplog.text
    assert (
        f"failed to remove generated material delivery path={delivery_path}"
        in captured_log
    )


def test_stop_cleanup_removes_demonstration_rawparts():
    stage = _FakeStage()
    adapter = _adapter(_FakeSidecar(), stage)
    demonstration_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Demonstration_test"
    )
    stage.paths.add(demonstration_path)
    adapter._demonstration_paths.add(demonstration_path)

    assert adapter.reset_generated_materials() is True
    assert stage.paths == set()
    assert adapter._demonstration_paths == set()


def test_demonstration_load_is_limited_to_once_per_play_to_stop_session(monkeypatch):
    state = {"playing": False}
    timeline = types.SimpleNamespace(is_playing=lambda: state["playing"])
    adapter = _adapter(_FakeSidecar(), timeline=timeline)
    created = []
    monkeypatch.setattr(
        adapter,
        "_create_demonstration_parts",
        lambda parts: created.append(parts) or len(parts),
    )
    try:
        adapter.load_demonstration("single")
    except RuntimeError as error:
        assert "Timeline Play" in str(error)
    else:
        raise AssertionError("Demonstration load must require Play")

    state["playing"] = True
    assert adapter.load_demonstration("single") == 1
    state["playing"] = False  # Pause does not alter the latch.
    state["playing"] = True  # Resume keeps the same latch.
    try:
        adapter.load_demonstration("side_by_side")
    except RuntimeError as error:
        assert "already loaded" in str(error)
    else:
        raise AssertionError("A Play session must accept at most one load")

    state["playing"] = False
    assert adapter.reset_generated_materials() is True
    state["playing"] = True
    assert adapter.load_demonstration("side_by_side") == 2
    assert len(created) == 2


def test_demonstration_availability_rejects_existing_or_pending_supply():
    timeline = types.SimpleNamespace(is_playing=lambda: True)
    adapter = _adapter(_FakeSidecar(), timeline=timeline)
    assert adapter.demonstration_load_unavailable_reason() is None

    adapter._runs["active-delivery"] = object()
    assert "empty Material Supply" in adapter.demonstration_load_unavailable_reason()
    adapter._runs.clear()

    adapter._pending_parts.append(("active-delivery", 0, None, 0))
    assert "empty Material Supply" in adapter.demonstration_load_unavailable_reason()


def test_stop_cleanup_removes_generated_prim_and_retires_delivery_identity():
    stage = _FakeStage()
    adapter = _adapter(_FakeSidecar(), stage)
    delivery_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_" + "02" * 16
    )
    stage.paths.add(delivery_path)
    adapter._delivery_paths.add(delivery_path)
    adapter._deliveries.add("02" * 16)
    adapter._pending_parts.append(("02" * 16, 0, None, 1))

    adapter.reset_generated_materials()

    assert delivery_path not in stage.paths
    assert not adapter._delivery_paths
    assert adapter._deliveries == {"02" * 16}
    assert not adapter._pending_parts


def test_stop_cleanup_detaches_before_removal_and_cancels_incomplete_delivery(
    monkeypatch,
):
    events = []

    class OrderedStage(_FakeStage):
        def RemovePrim(self, path):
            events.append("material")
            super().RemovePrim(path)

    delivery_id = "03" * 16
    delivery_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_" + delivery_id
    )
    part_path = delivery_path + "/RawPart_0"
    stage = OrderedStage()
    stage.paths.update({delivery_path, part_path})
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 50)
    adapter._delivery_paths.add(delivery_path)
    adapter._deliveries.add(delivery_id)
    adapter._runs[delivery_id] = MaterialSupplyRun(
        delivery_id=delivery_id,
        material_type="RawPart",
        requested_quantity=1,
        deadline_ns=100,
    )
    adapter._runs[delivery_id].record_supplied(part_path, completed_ns=50)

    result = adapter.reset_generated_materials(
        lambda: events.append("attachment") or True
    )

    assert result is True
    assert events == ["attachment", "material"]
    assert sidecar.sent == [
        {
            "delivery_id": delivery_id,
            "material_type": "RawPart",
            "requested_quantity": 1,
            "supplied_quantity": 1,
            "completed": False,
            "valid": True,
            "failure_reason": "CANCELED",
            "uncompleted_spawn_count": 0,
            "execution_error": None,
        }
    ]
    assert adapter._stop_reset_pending is False
    assert adapter.reset_generated_materials() is True
    assert len(sidecar.sent) == 1
    adapter._on_accepted_trigger(
        {
            "delivery_id": delivery_id,
            "material_type": "RawPart",
            "quantity": 1,
            "valid": True,
            "has_random_seed": False,
        }
    )
    assert adapter._pending_parts == []
    assert delivery_id not in adapter._runs


def test_stop_cleanup_failure_keeps_material_and_work_for_lifecycle_retry():
    stage = _FakeStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    delivery_id = "04" * 16
    delivery_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_" + delivery_id
    )
    stage.paths.add(delivery_path)
    adapter._delivery_paths.add(delivery_path)
    adapter._deliveries.add(delivery_id)
    adapter._runs[delivery_id] = MaterialSupplyRun(
        delivery_id=delivery_id,
        material_type="RawPart",
        requested_quantity=1,
        deadline_ns=100,
    )

    assert adapter.reset_generated_materials(lambda: False) is False

    assert delivery_path in stage.paths
    assert adapter._runs[delivery_id].terminal_result is None
    assert sidecar.sent == []
    assert adapter._stop_reset_pending is True


def test_delivery_metadata_uses_supported_unsigned_value_type_names(monkeypatch):
    class _Attribute:
        def __init__(self, name, value_type):
            self.name = name
            self.value_type = value_type

        def Set(self, value):
            pass

    class _Prim:
        def __init__(self):
            self.attributes = {}

        def CreateAttribute(self, name, value_type, custom):
            attribute = _Attribute(name, value_type)
            self.attributes[name] = attribute
            return attribute

    class _Xform:
        @staticmethod
        def Define(stage, path):
            stage.defined_path = path
            return types.SimpleNamespace(GetPrim=lambda: stage.prim)

    value_types = types.SimpleNamespace(
        String="string", UInt="uint", UInt64="uint64", Bool="bool"
    )
    pxr = types.SimpleNamespace(
        Sdf=types.SimpleNamespace(ValueTypeNames=value_types),
        UsdGeom=types.SimpleNamespace(Xform=_Xform),
    )
    monkeypatch.setitem(sys.modules, "pxr", pxr)
    stage = types.SimpleNamespace(prim=_Prim())
    adapter = _adapter(_FakeSidecar(), stage)

    adapter._author_delivery_metadata("46356a2ddcfe463fac9eb473f3ba0bf2", 1, 7, False)

    assert stage.prim.attributes["sf_twin:requested_quantity"].value_type == "uint"


def test_duplicate_accepted_delivery_does_not_schedule_a_second_quantity(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 100)
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *args, **kwargs: iter([(0, 0, 0, 0, 0, 0)] * 2),
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *args: None)
    trigger = {
        "delivery_id": "01" * 16,
        "stamp_ns": 100,
        "material_type": "RawPart",
        "quantity": 2,
        "valid": True,
        "has_random_seed": True,
        "random_seed": 42,
    }

    adapter._on_accepted_trigger(trigger)
    adapter._on_accepted_trigger(trigger)

    assert len(adapter._pending_parts) == 2
    assert adapter._runs["01" * 16].requested_quantity == 2


def test_deadline_terminal_cancels_remaining_spawns_before_they_can_create_parts(
    monkeypatch, caplog
):
    delivery_id = "51" * 16
    part_path = (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_"
        + delivery_id
        + "/RawPart_0000"
    )
    stage = _ObservedStage()
    stage.paths.add(part_path)
    stage.positions[part_path] = (0.0, 0.0, 0.05)
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 100)
    _install_fake_usd(monkeypatch, stage)
    observation = MaterialSupplyRun(
        delivery_id=delivery_id,
        material_type="RawPart",
        requested_quantity=2,
        deadline_ns=100,
    )
    observation.record_supplied(part_path, completed_ns=50)
    adapter._runs[delivery_id] = observation
    adapter._pending_parts.extend(
        [
            (delivery_id, 1, (0, 0, 0, 0, 0, 0), 99),
        ]
    )
    created = []
    monkeypatch.setattr(adapter, "_create_part", lambda *args: created.append(args))

    adapter._spawn_one_if_due()
    adapter._finish_deliveries()

    assert created == []
    assert adapter._pending_parts == []
    assert delivery_id not in adapter._runs
    assert len(sidecar.sent) == 1
    assert sidecar.sent[0]["failure_reason"] == "SUPPLY"
    terminal_log = caplog.text
    assert "uncompleted_spawn_count=1" in terminal_log
    assert "simulation_time_ns=100" in terminal_log
    assert "deadline_ns=100" in terminal_log


def test_spawn_discards_orphaned_or_already_terminal_delivery_work(monkeypatch):
    delivery_id = "57" * 16
    for active_observation in (False, True):
        adapter = _adapter(_FakeSidecar())
        if active_observation:
            observation = MaterialSupplyRun(
                delivery_id=delivery_id,
                material_type="RawPart",
                requested_quantity=2,
                deadline_ns=100,
            )
            observation.terminal_result = {
                "delivery_id": delivery_id,
                "material_type": "RawPart",
                "requested_quantity": 2,
                "supplied_quantity": 1,
                "completed": False,
                "valid": True,
                "failure_reason": "SUPPLY",
            }
            adapter._runs[delivery_id] = observation
        adapter._pending_parts.append((delivery_id, 1, (0, 0, 0, 0, 0, 0), 0))
        created = []
        monkeypatch.setattr(
            adapter,
            "_create_part",
            lambda *args, _created=created: _created.append(args),
        )

        adapter._spawn_one_if_due()

        assert created == []
        assert adapter._pending_parts == []


def test_first_part_leaving_handoff_does_not_cancel_second_spawn(monkeypatch):
    import scene_builder.material_supply_adapter as adapter_module

    delivery_id = "56" * 16
    stage = _ObservedStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    now = [50]
    monkeypatch.setattr(adapter, "_now_ns", lambda: now[0])
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *_args, **_kwargs: [(0, 0, 0, 0, 0, 0)] * 2,
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *_args: None)
    adapter._on_accepted_trigger(
        {
            "delivery_id": delivery_id,
            "material_type": "RawPart",
            "quantity": 2,
            "valid": True,
            "has_random_seed": False,
        }
    )
    created = []

    def create_part(identity, index, _pose):
        path = f"/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_{identity}/RawPart_{index:04d}"
        created.append(path)
        stage.positions[path] = (0.0, 0.0, 0.05)
        stage.paths.add(path)
        return path

    monkeypatch.setattr(adapter, "_create_part", create_part)
    adapter._spawn_one_if_due()
    adapter._finish_deliveries()
    stage.positions[created[0]] = (10.0, 0.0, 0.05)
    now[0] += 10_000_000
    adapter._spawn_one_if_due()
    adapter._finish_deliveries()

    assert len(created) == 2
    assert len(sidecar.sent) == 1
    assert sidecar.sent[0]["completed"] is True
    assert sidecar.sent[0]["supplied_quantity"] == 2


def test_spawn_failure_terminates_delivery_and_clears_its_remaining_work(
    monkeypatch, caplog
):
    delivery_id = "54" * 16
    stage = _ObservedStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 50)
    _install_fake_usd(monkeypatch, stage)
    adapter._runs[delivery_id] = MaterialSupplyRun(
        delivery_id=delivery_id,
        material_type="RawPart",
        requested_quantity=2,
        deadline_ns=100,
    )
    adapter._pending_parts.extend(
        [
            (delivery_id, 0, (0, 0, 0, 0, 0, 0), 50),
            (delivery_id, 1, (0, 0, 0, 0, 0, 0), 60),
        ]
    )

    def fail_spawn(*_args):
        raise RuntimeError("USD authoring failed")

    monkeypatch.setattr(adapter, "_create_part", fail_spawn)
    adapter._spawn_one_if_due()
    adapter._finish_deliveries()

    assert adapter._pending_parts == []
    assert delivery_id not in adapter._runs
    assert len(sidecar.sent) == 1
    assert sidecar.sent[0]["supplied_quantity"] == 0
    assert sidecar.sent[0]["failure_reason"] == "SUPPLY"
    assert "uncompleted_spawn_count=2" in caplog.text


def test_quantity_one_normal_generation_completes_once(monkeypatch):
    import scene_builder.material_supply_adapter as adapter_module

    delivery_id = "55" * 16
    stage = _ObservedStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 100)
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *_args, **_kwargs: [(0, 0, 0, 0, 0, 0)],
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *_args: None)
    _install_fake_usd(monkeypatch, stage)

    adapter._on_accepted_trigger(
        {
            "delivery_id": delivery_id,
            "material_type": "RawPart",
            "quantity": 1,
            "valid": True,
            "has_random_seed": False,
        }
    )
    path = f"/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_{delivery_id}/RawPart_0000"
    monkeypatch.setattr(
        adapter,
        "_create_part",
        lambda *_args: stage.paths.add(path) or path,
    )
    stage.positions[path] = (0.0, 0.0, 0.05)

    adapter._spawn_one_if_due()
    adapter._finish_deliveries()
    adapter._finish_deliveries()

    assert delivery_id not in adapter._runs
    assert adapter._pending_parts == []
    assert len(sidecar.sent) == 1
    assert sidecar.sent[0]["completed"] is True


def test_quantity_two_normal_generation_remains_active_until_both_parts_arrive(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    delivery_id = "52" * 16
    stage = _ObservedStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    now = [100]
    monkeypatch.setattr(adapter, "_now_ns", lambda: now[0])
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *_args, **_kwargs: [(0, 0, 0, 0, 0, 0)] * 2,
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *_args: None)
    _install_fake_usd(monkeypatch, stage)

    adapter._on_accepted_trigger(
        {
            "delivery_id": delivery_id,
            "material_type": "RawPart",
            "quantity": 2,
            "valid": True,
            "has_random_seed": False,
        }
    )

    paths = []

    def create_part(identity, index, _pose):
        path = f"/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_{identity}/RawPart_{index:04d}"
        paths.append(path)
        stage.paths.add(path)
        stage.positions[path] = (0.0, 0.0, 0.05)
        return path

    monkeypatch.setattr(adapter, "_create_part", create_part)
    adapter._spawn_one_if_due()
    adapter._finish_deliveries()
    assert len(paths) == 1
    assert adapter._runs[delivery_id].terminal_result is None

    now[0] += 10_000_000
    adapter._spawn_one_if_due()
    adapter._finish_deliveries()

    assert len(paths) == 2
    assert delivery_id not in adapter._runs
    assert adapter._pending_parts == []
    assert len(sidecar.sent) == 1
    assert sidecar.sent[0]["completed"] is True


def test_distinct_deliveries_add_material_and_stop_removes_only_delivery_prims(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    stage = _ObservedStage()
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar, stage)
    monkeypatch.setattr(adapter, "_now_ns", lambda: 100)
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda _profile, quantity, **_kwargs: [(0, 0, 0, 0, 0, 0)] * quantity,
    )
    monkeypatch.setattr(
        adapter,
        "_author_delivery_metadata",
        lambda delivery_id, *_: stage.paths.add(
            adapter_module._delivery_prim_path(delivery_id)
        ),
    )
    unrelated_path = "/World/SF_Twin_Cell/MaterialSupply/Unrelated"
    stage.paths.add(unrelated_path)

    first_id = "31" * 16
    second_id = "32" * 16
    for delivery_id, quantity in ((first_id, 1), (second_id, 2)):
        adapter._on_accepted_trigger(
            {
                "delivery_id": delivery_id,
                "material_type": "RawPart",
                "quantity": quantity,
                "valid": True,
                "has_random_seed": False,
            }
        )

    first_path = adapter_module._delivery_prim_path(first_id)
    second_path = adapter_module._delivery_prim_path(second_id)
    assert {first_path, second_path, unrelated_path} == stage.paths
    assert len(adapter._pending_parts) == 3
    assert set(adapter._runs) == {first_id, second_id}

    adapter.reset_generated_materials()

    assert stage.paths == {unrelated_path}
    assert adapter._deliveries == {"31" * 16, "32" * 16}
    assert not adapter._pending_parts


def test_delivery_already_in_stage_remains_duplicate_after_adapter_restart(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    stage = _FakeStage()
    first = _adapter(_FakeSidecar(), stage)
    second = _adapter(_FakeSidecar(), stage)
    monkeypatch.setattr(first, "_now_ns", lambda: 100)
    monkeypatch.setattr(second, "_now_ns", lambda: 100)
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *args, **kwargs: iter([(0, 0, 0, 0, 0, 0)]),
    )
    monkeypatch.setattr(
        first,
        "_author_delivery_metadata",
        lambda delivery_id, *_: stage.paths.add(
            adapter_module._delivery_prim_path(delivery_id)
        ),
    )
    trigger = {
        "delivery_id": "02" * 16,
        "stamp_ns": 100,
        "material_type": "RawPart",
        "quantity": 1,
        "valid": True,
        "has_random_seed": False,
    }

    first._on_accepted_trigger(trigger)
    second._on_accepted_trigger(trigger)

    assert len(first._pending_parts) == 1
    assert second._pending_parts == []
    assert second._runs == {}


def test_adapter_does_not_revalidate_integration_accepted_source_timestamp(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    adapter = _adapter(_FakeSidecar())
    monkeypatch.setattr(adapter, "_now_ns", lambda: 100)
    monkeypatch.setattr(
        adapter_module,
        "sample_initial_poses",
        lambda *args, **kwargs: [(0, 0, 0, 0, 0, 0)],
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *args: None)

    adapter._on_accepted_trigger(
        {
            "delivery_id": "01" * 16,
            "stamp_ns": 10_000,
            "material_type": "RawPart",
            "quantity": 1,
            "valid": True,
            "has_random_seed": False,
        }
    )

    assert adapter._deliveries == {"01" * 16}
    assert adapter._runs["01" * 16].deadline_ns == 1_000_000_100
    assert len(adapter._pending_parts) == 1


def test_provided_seed_repeats_exactly_the_requested_pose_quantity():
    profile = _adapter(_FakeSidecar()).profile

    first = list(
        sample_initial_poses(profile, 3, shelf_origin_m=(0.0, 0.0, 0.0), seed=42)
    )
    second = list(
        sample_initial_poses(profile, 3, shelf_origin_m=(0.0, 0.0, 0.0), seed=42)
    )

    assert len(first) == 3
    assert first == second


def test_sidecar_loss_and_adapter_shutdown_drop_incomplete_work_without_success():
    sidecar = _FakeSidecar()
    adapter = _adapter(sidecar)
    adapter._pending_parts.append(("delivery-a", 0, (0, 0, 0, 0, 0, 0), 0))
    adapter._runs["delivery-a"] = object()
    sidecar.available = False

    adapter.spin_once()

    assert adapter._pending_parts == []
    assert adapter._runs == {}
    assert sidecar.sent == []
    adapter.close()
    adapter.close()
    assert sidecar.closed
    assert sidecar.sent == []


def test_trigger_wire_payload_preserves_seed_and_ros_source_stamp():
    payload = trigger_payload(
        delivery_id=bytes(range(1, 17)),
        stamp_ns=1_234_000_005,
        material_type="RawPart",
        quantity=3,
        valid=True,
        has_random_seed=True,
        random_seed=987654321,
    )

    assert payload == {
        "op": "accepted_handoff",
        "delivery_id": bytes(range(1, 17)).hex(),
        "stamp_ns": 1_234_000_005,
        "material_type": "RawPart",
        "quantity": 3,
        "valid": True,
        "has_random_seed": True,
        "random_seed": 987654321,
    }


def test_adapter_forwards_integration_accepted_event_without_timestamp_gate(
    monkeypatch,
):
    import scene_builder.material_supply_adapter as adapter_module

    adapter = _adapter(_FakeSidecar())
    monkeypatch.setattr(adapter, "_now_ns", lambda: 12_150_000_000)
    monkeypatch.setattr(
        adapter_module, "sample_initial_poses", lambda *a, **k: [(0, 0, 0, 0, 0, 0)]
    )
    monkeypatch.setattr(adapter, "_author_delivery_metadata", lambda *args: None)

    adapter._on_accepted_trigger(
        {
            "delivery_id": "04" * 16,
            "stamp_ns": 12_150_000_633,
            "material_type": "RawPart",
            "quantity": 1,
            "valid": True,
            "has_random_seed": False,
        }
    )

    assert len(adapter._pending_parts) == 1
    assert adapter._runs["04" * 16].deadline_ns == 13_150_000_000


def test_completion_wire_payload_preserves_delivery_counts_and_failure():
    payload = completion_payload(
        {
            "delivery_id": "010203",
            "material_type": "RawPart",
            "requested_quantity": 4,
            "supplied_quantity": 4,
            "completed": False,
            "valid": True,
            "failure_reason": "SUPPLY",
        }
    )

    assert payload["op"] == "transfer_completion"
    assert payload["result"]["delivery_id"] == "010203"
    assert payload["result"]["failure_reason"] == "SUPPLY"
    assert set(payload["result"]) == {
        "delivery_id",
        "material_type",
        "requested_quantity",
        "supplied_quantity",
        "completed",
        "valid",
        "failure_reason",
    }


def test_canceled_completion_maps_to_the_additive_transfer_result_code():
    class Message:
        TRANSFER_FAILURE_NONE = 0
        TRANSFER_FAILURE_SUPPLY = 1
        TRANSFER_FAILURE_CANCELED = 4

        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.delivery_id = types.SimpleNamespace(uuid=None)

    result = _transfer_completion_message(
        {
            "delivery_id": "05" * 16,
            "material_type": "RawPart",
            "requested_quantity": 2,
            "supplied_quantity": 0,
            "completed": False,
            "valid": True,
            "failure_reason": "CANCELED",
        },
        Message,
        observed_at="observed",
    )

    assert result.failure_reason == Message.TRANSFER_FAILURE_CANCELED
    assert result.supplied_quantity == 0


def test_sidecar_publishes_at_most_one_completion_per_delivery():
    published = set()
    result = {"delivery_id": "01" * 16, "completed": True}

    assert _accept_completion_once(result, published)
    assert not _accept_completion_once(result, published)


def test_client_buffers_partial_packets_and_keeps_duplicate_events_visible():
    parent, child = socket.socketpair()
    parent.setblocking(False)
    client = MaterialSupplySidecarClient.from_socket(parent, process=None)
    try:
        packet = (
            json.dumps(
                {"op": "accepted_handoff", "delivery_id": "delivery-a", "quantity": 2}
            ).encode()
            + b"\n"
        )
        child.sendall(packet[:13])
        assert client.poll_messages() == []
        child.sendall(packet[13:] + packet)
        assert client.poll_messages() == [
            {"op": "accepted_handoff", "delivery_id": "delivery-a", "quantity": 2},
            {"op": "accepted_handoff", "delivery_id": "delivery-a", "quantity": 2},
        ]
    finally:
        client.close()
        child.close()


def test_client_marks_peer_disconnect_as_unavailable():
    parent, child = socket.socketpair()
    parent.setblocking(False)
    client = MaterialSupplySidecarClient.from_socket(parent, process=None)
    child.close()

    assert client.poll_messages() == []
    assert not client.available
    client.close()


def test_client_sends_completion_without_blocking():
    parent, child = socket.socketpair()
    parent.setblocking(False)
    child.setblocking(False)
    client = MaterialSupplySidecarClient.from_socket(parent, process=None)
    result = {
        "delivery_id": "delivery-a",
        "material_type": "RawPart",
        "requested_quantity": 1,
        "supplied_quantity": 1,
        "completed": True,
        "valid": True,
        "failure_reason": "NONE",
    }
    try:
        client.send_completion(result)
        client.flush_outgoing()
        received = child.recv(4096)
        assert json.loads(received)["result"] == result
    finally:
        client.close()
        child.close()


def test_shutdown_closes_ipc_without_synthesizing_completion():
    parent, child = socket.socketpair()
    parent.setblocking(False)
    child.setblocking(False)
    client = MaterialSupplySidecarClient.from_socket(parent, process=None)

    client.close()

    assert child.recv(4096) == b""
    child.close()
