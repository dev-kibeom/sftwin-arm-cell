from types import SimpleNamespace

from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension


def test_responsive_layout_switches_at_wide_window_breakpoint():
    assert SimulationUIHubExtension._layout_mode_for_width(900) == "wide"
    assert SimulationUIHubExtension._layout_mode_for_width(779) == "narrow"


def test_camera_display_fits_width_and_preserves_source_aspect_ratio():
    for width in (180, 420, 900):
        image_width, image_height = SimulationUIHubExtension._camera_size_for_width(
            width
        )
        assert image_width <= max(72, width - 36)
        assert image_width / image_height == 16 / 9


def test_window_resize_updates_image_size_without_rebuilding_same_layout():
    class Pixel:
        def __init__(self, value):
            self.value = value

    class Image:
        @property
        def width(self):
            return self._width

        @width.setter
        def width(self, value):
            assert isinstance(value, Pixel)
            self._width = value

        @property
        def height(self):
            return self._height

        @height.setter
        def height(self, value):
            assert isinstance(value, Pixel)
            self._height = value

    extension = SimulationUIHubExtension()
    extension._ui = SimpleNamespace(Pixel=Pixel)
    extension._layout_mode = "narrow"
    extension._camera_image = Image()
    extension._window = SimpleNamespace(frame=SimpleNamespace(rebuild=lambda: None))

    extension._on_window_width_changed(420)

    assert extension._camera_image.width.value == 384
    assert extension._camera_image.height.value == 216


def test_crossing_layout_breakpoint_rebuilds_ui_once():
    rebuilds = []
    extension = SimulationUIHubExtension()
    extension._layout_mode = "narrow"
    extension._window = SimpleNamespace(
        frame=SimpleNamespace(rebuild=lambda: rebuilds.append(True))
    )

    extension._on_window_width_changed(900)
    extension._on_window_width_changed(920)

    assert extension._layout_mode == "wide"
    assert rebuilds == [True]
