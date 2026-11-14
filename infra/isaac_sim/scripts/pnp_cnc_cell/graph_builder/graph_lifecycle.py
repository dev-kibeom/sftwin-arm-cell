"""Small OmniGraph lifecycle operations shared by Isaac entrypoints."""


def release_graph_controller_node(controller, graph_path: str) -> None:
    """Release the articulation controller's cached PhysX view via OmniGraph."""
    controller.delete_node(
        node_id=f"{graph_path}/ArticulationController",
        ignore_if_missing=True,
        undoable=False,
    )
