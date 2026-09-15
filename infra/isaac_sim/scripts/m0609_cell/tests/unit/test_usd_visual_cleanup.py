import pytest

Sdf = pytest.importorskip("pxr.Sdf")

from usd_visual_cleanup import cleanup_dangling_visual_references


ROBOT = "/m0609_robotiq_2f85"


def visual_reference(layer, link, target_exists):
    visual = Sdf.CreatePrimInLayer(layer, f"{ROBOT}/{link}/visuals")
    visual.referenceList.prependedItems = [Sdf.Reference("", f"/visuals/{link}")]
    if target_exists:
        Sdf.CreatePrimInLayer(layer, f"/visuals/{link}")
    return visual


def test_visual_less_link_removes_only_missing_visual_reference():
    layer = Sdf.Layer.CreateAnonymous()
    visual = visual_reference(layer, "semantic_frame", target_exists=False)

    cleaned = cleanup_dangling_visual_references(
        layer, ROBOT, {"semantic_frame": False}
    )

    assert cleaned == ["semantic_frame"]
    assert visual.referenceList.prependedItems == []


def test_visual_required_link_with_missing_target_fails_without_cleanup():
    layer = Sdf.Layer.CreateAnonymous()
    visual = visual_reference(layer, "visible_link", target_exists=False)

    with pytest.raises(RuntimeError, match="visual geometry.*visible_link"):
        cleanup_dangling_visual_references(layer, ROBOT, {"visible_link": True})
    assert visual.referenceList.prependedItems == [
        Sdf.Reference("", "/visuals/visible_link")
    ]


def test_valid_visual_link_preserves_reference():
    layer = Sdf.Layer.CreateAnonymous()
    visual = visual_reference(layer, "visible_link", target_exists=True)

    cleaned = cleanup_dangling_visual_references(
        layer, ROBOT, {"visible_link": True}
    )

    assert cleaned == []
    assert visual.referenceList.prependedItems == [
        Sdf.Reference("", "/visuals/visible_link")
    ]
