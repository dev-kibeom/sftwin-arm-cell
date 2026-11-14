import numpy as np

from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension


def _metadata():
    return {
        "object_region": (10, 10, 10, 10),
        "support_region": [(30, 10), (40, 10), (35, 20)],
    }


def test_depth_annotation_adds_contrasting_outline_without_changing_geometry():
    pixels = np.full((32, 48, 4), (100, 100, 100, 255), dtype=np.uint8)

    SimulationUIHubExtension._annotate_vision(
        pixels, _metadata(), display_source="depth"
    )

    assert np.array_equal(pixels[10, 15], (32, 255, 96, 255))
    assert np.array_equal(pixels[10, 34], (255, 192, 32, 255))
    assert np.array_equal(pixels[7, 15], (12, 12, 12, 255))
    assert np.array_equal(pixels[7, 35], (12, 12, 12, 255))


def test_rgb_annotation_keeps_existing_appearance_without_depth_outline():
    pixels = np.full((32, 48, 4), (100, 100, 100, 255), dtype=np.uint8)

    SimulationUIHubExtension._annotate_vision(pixels, _metadata(), display_source="rgb")

    assert np.array_equal(pixels[10, 15], (32, 255, 96, 255))
    assert np.array_equal(pixels[10, 34], (255, 192, 32, 255))
    assert np.array_equal(pixels[7, 15], (100, 100, 100, 255))
