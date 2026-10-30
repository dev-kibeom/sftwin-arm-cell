"""Python 3.10 ROS endpoint for the Isaac Material Supply adapter."""

from __future__ import annotations

import json
import select
import socket
import sys
from collections import deque


def trigger_payload(
    *,
    delivery_id,
    stamp_ns,
    material_type,
    quantity,
    valid,
    has_random_seed,
    random_seed,
):
    """Convert the ROS trigger to the private local wire format."""
    if len(delivery_id) != 16:
        raise ValueError("delivery_id must contain 16 UUID bytes")
    payload = {
        "op": "accepted_handoff",
        "delivery_id": delivery_id.hex(),
        "stamp_ns": int(stamp_ns),
        "material_type": str(material_type),
        "quantity": int(quantity),
        "valid": bool(valid),
        "has_random_seed": bool(has_random_seed),
    }
    if has_random_seed:
        payload["random_seed"] = int(random_seed)
    return payload


def _uuid_bytes(value):
    raw = bytes.fromhex(str(value))
    if len(raw) != 16:
        raise ValueError("delivery_id must encode 16 UUID bytes")
    return raw


def _accept_completion_once(result, published_deliveries):
    delivery_id = str(result["delivery_id"])
    if delivery_id in published_deliveries:
        return False
    published_deliveries.add(delivery_id)
    return True


def _transfer_completion_message(result, message_type, observed_at):
    """Build the public IDL message from an Isaac provider completion fact."""
    message = message_type()
    message.header.stamp = observed_at
    message.delivery_id.uuid = list(_uuid_bytes(result["delivery_id"]))
    message.material_type = str(result["material_type"])
    message.requested_quantity = int(result["requested_quantity"])
    message.supplied_quantity = int(result["supplied_quantity"])
    message.completed = bool(result["completed"])
    message.valid = bool(result["valid"])
    failure_reasons = {
        "NONE": message.TRANSFER_FAILURE_NONE,
        "SUPPLY": message.TRANSFER_FAILURE_SUPPLY,
        "CANCELED": message.TRANSFER_FAILURE_CANCELED,
    }
    message.failure_reason = failure_reasons[str(result["failure_reason"])]
    return message


def _run(fd):
    import rclpy
    from arm_cell_interfaces.msg import HandoffTrigger, TransferCompletion
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    sock = socket.socket(fileno=fd)
    sock.setblocking(False)
    pending = deque()
    outgoing = bytearray()

    class MaterialSupplyRosNode(Node):
        def __init__(self):
            super().__init__(
                "isaac_material_supply_sidecar",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.transport_overflow = False
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.completion_publisher = self.create_publisher(
                TransferCompletion, "/material_supply/transfer_completion", qos
            )
            self.create_subscription(
                HandoffTrigger,
                "/integration/accepted_handoff",
                self.accepted_handoff,
                qos,
            )

        def accepted_handoff(self, message):
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            delivery_id = bytes(message.delivery_id.uuid).hex()
            if len(pending) >= 64:
                self.transport_overflow = True
                self.get_logger().error(
                    "material supply IPC backlog exceeded 64 accepted events"
                )
                return
            payload = trigger_payload(
                delivery_id=bytes(message.delivery_id.uuid),
                stamp_ns=stamp_ns,
                material_type=message.material_type,
                quantity=message.quantity,
                valid=message.valid,
                has_random_seed=message.has_random_seed,
                random_seed=message.random_seed,
            )
            pending.append((json.dumps(payload, separators=(",", ":")) + "\n").encode())

        def publish_completion(self, result):
            message = _transfer_completion_message(
                result, TransferCompletion, self.get_clock().now().to_msg()
            )
            self.completion_publisher.publish(message)

    if not rclpy.ok():
        rclpy.init(args=None)
    node = MaterialSupplyRosNode()
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    receive_buffer = bytearray()
    published_deliveries = set()
    try:
        sock.setblocking(True)
        sock.sendall(b'{"op":"ready","ready":true}\n')
        sock.setblocking(False)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.005)
            if node.transport_overflow:
                break
            readable, writable, _ = select.select(
                [sock], [sock] if pending or outgoing else [], [], 0
            )
            if readable:
                try:
                    chunk = sock.recv(65536)
                except BlockingIOError:
                    chunk = None
                if chunk == b"":
                    break
                if chunk:
                    receive_buffer.extend(chunk)
                    while b"\n" in receive_buffer:
                        line, _, rest = receive_buffer.partition(b"\n")
                        receive_buffer = bytearray(rest)
                        try:
                            command = json.loads(line)
                            if command.get("op") == "transfer_completion":
                                result = command["result"]
                                if _accept_completion_once(
                                    result, published_deliveries
                                ):
                                    node.publish_completion(result)
                        except (ValueError, KeyError, TypeError) as error:
                            node.get_logger().warning(
                                f"Material completion command rejected: {error}"
                            )
            if pending:
                outgoing.extend(pending.popleft())
            if writable and outgoing:
                try:
                    sent = sock.send(outgoing)
                    del outgoing[:sent]
                except BlockingIOError:
                    pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sock.close()


if __name__ == "__main__":
    try:
        _run(int(sys.argv[1]))
    except Exception as error:
        fd = int(sys.argv[1])
        try:
            sock = socket.socket(fileno=fd)
            failure = {"op": "ready", "ready": False, "error": str(error)}
            sock.sendall((json.dumps(failure) + "\n").encode())
            sock.close()
        except OSError:
            pass
        raise
