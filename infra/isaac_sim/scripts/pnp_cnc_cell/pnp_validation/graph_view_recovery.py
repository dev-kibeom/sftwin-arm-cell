"""Recreate the Isaac command graph around physics view invalidation."""

from __future__ import annotations

from typing import Callable, TypeVar


Result = TypeVar("Result")


def delete_with_graph_rebuild(
    *,
    stage,
    graph_path: str,
    release_graph_view: Callable[[], None],
    delete_fixture: Callable[[], Result],
    rebuild_graph: Callable[[], None],
) -> Result:
    """Release the graph controller node before deleting a fixture prim.

    Isaac's ArticulationController owns a physics tensor view in its OmniGraph
    node instance. Delete the node through OmniGraph so its release_instance
    lifecycle drops the cached view before the USD physics scene changes. The
    graph is restored after the stage mutation, including when fixture
    deletion fails.
    """
    graph_prim = stage.GetPrimAtPath(graph_path)
    if graph_prim.IsValid():
        release_graph_view()
    try:
        return delete_fixture()
    finally:
        rebuild_graph()
