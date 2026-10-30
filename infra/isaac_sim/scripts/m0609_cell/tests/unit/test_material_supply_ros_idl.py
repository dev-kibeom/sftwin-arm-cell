import json
import socket

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("arm_cell_interfaces")

from arm_cell_interfaces.msg import TransferCompletion
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message

from material_supply_ros_sidecar import _transfer_completion_message
from material_supply_ros_sidecar import trigger_payload
from scene_builder.material_supply_sidecar import MaterialSupplySidecarClient


@pytest.mark.parametrize(
    ("failure_reason", "constant_name", "completed", "valid", "supplied"),
    (
        ("NONE", "TRANSFER_FAILURE_NONE", True, True, 3),
        ("SUPPLY", "TRANSFER_FAILURE_SUPPLY", False, True, 1),
        ("CANCELED", "TRANSFER_FAILURE_CANCELED", False, True, 1),
    ),
)
def test_provider_neutral_completion_fields_survive_ipc_serialization(
    failure_reason, constant_name, completed, valid, supplied
):
    fields = set(TransferCompletion().get_fields_and_field_types())
    assert fields == {
        "header",
        "delivery_id",
        "material_type",
        "requested_quantity",
        "supplied_quantity",
        "completed",
        "valid",
        "failure_reason",
    }

    result = {
        "delivery_id": bytes(range(1, 17)).hex(),
        "material_type": "RawPart",
        "requested_quantity": 3,
        "supplied_quantity": supplied,
        "completed": completed,
        "valid": valid,
        "failure_reason": failure_reason,
    }
    parent, child = socket.socketpair()
    client = MaterialSupplySidecarClient.from_socket(parent)
    try:
        trigger = trigger_payload(
            delivery_id=bytes(range(1, 17)),
            stamp_ns=1_234,
            material_type="RawPart",
            quantity=3,
            valid=True,
            has_random_seed=True,
            random_seed=42,
        )
        child.sendall((json.dumps(trigger) + "\n").encode())
        assert client.poll_messages() == [trigger]

        client.send_completion(result)
        wire_result = json.loads(child.recv(4096))["result"]
    finally:
        client.close()
        child.close()

    ros_message = _transfer_completion_message(
        wire_result, TransferCompletion, Time(sec=17, nanosec=23)
    )
    published_message = deserialize_message(
        serialize_message(ros_message), TransferCompletion
    )

    assert published_message.header.stamp.sec == 17
    assert published_message.header.stamp.nanosec == 23
    assert bytes(published_message.delivery_id.uuid) == bytes(range(1, 17))
    assert published_message.material_type == "RawPart"
    assert published_message.requested_quantity == 3
    assert published_message.supplied_quantity == supplied
    assert published_message.completed is completed
    assert published_message.valid is valid
    assert published_message.failure_reason == getattr(
        TransferCompletion, constant_name
    )
