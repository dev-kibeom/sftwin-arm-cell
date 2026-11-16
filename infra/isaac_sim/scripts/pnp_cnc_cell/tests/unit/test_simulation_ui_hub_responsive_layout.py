from types import SimpleNamespace

import pytest

from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension


def test_layout_adapts_between_narrow_docking_and_wide_window():
    assert SimulationUIHubExtension._layout_mode_for_width(180) == "narrow"
    assert SimulationUIHubExtension._layout_mode_for_width(1200) == "wide"


def test_camera_display_fits_width_and_preserves_aspect_ratio():
    sizes = [SimulationUIHubExtension._camera_size_for_width(width)
             for width in (180, 420, 900)]
    for available_width, (image_width, image_height) in zip((180, 420, 900), sizes):
        assert 0 < image_width <= available_width
        assert image_height > 0
        assert image_width / image_height == pytest.approx(16 / 9)
    assert sizes[0][0] < sizes[-1][0]


def test_window_resize_updates_display_without_changing_video_identity():
    extension = SimulationUIHubExtension()
    extension._ui = SimpleNamespace(Pixel=lambda value: value)
    extension._layout_mode = "narrow"
    extension._camera_image = SimpleNamespace(width=0, height=0)
    extension._window = SimpleNamespace(frame=SimpleNamespace(rebuild=lambda: None))
    # Resize must not use runtime subscription/conversion operations.
    extension.runtime = SimpleNamespace()
    identity = ("raw_rgb", "camera_color_optical_frame", 42)
    extension._last_image_identity = identity

    widths = []
    for available_width in (180, 420, 1200, 180):
        extension._on_window_width_changed(available_width)
        image = extension._camera_image
        assert 0 < image.width <= available_width
        assert image.height > 0
        assert image.width / image.height == pytest.approx(16 / 9)
        assert extension._last_image_identity == identity
        widths.append(image.width)
    assert widths[0] < widths[2]
    assert widths[0] == widths[-1]
    assert extension._layout_mode == "narrow"
