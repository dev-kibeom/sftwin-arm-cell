from types import SimpleNamespace

import pytest

from rgbd_assertions import (
    coarse_depth_grid,
    decode_depth_frame,
    depth_coordinate_hypotheses,
    depth_frame_distribution,
    depth_patch_summary,
    depth_value_at,
    is_valid_depth,
    validate_camera_info,
    validate_image_payload,
)
from contract_config import acceptance_config


def image(encoding="rgb8", width=2, height=1, step=6, data=b"abcdef", big=False):
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=step,
        data=data,
        is_bigendian=big,
    )


def test_image_payload_accepts_padding_and_rejects_bad_layouts():
    assert (
        validate_image_payload(image(step=8, data=b"abcdefgh"))["row_padding_bytes"]
        == 2
    )
    with pytest.raises(ValueError):
        validate_image_payload(image(encoding="bgr8"))
    with pytest.raises(ValueError):
        validate_image_payload(image(step=5))
    with pytest.raises(ValueError):
        validate_image_payload(image(data=b"short"))


def test_depth_endianness_and_invalid_values_are_preserved():
    import struct

    for big in (False, True):
        payload = struct.pack((">" if big else "<") + "f", 1.25)
        message = image("32FC1", width=1, step=4, data=payload, big=big)
        assert depth_value_at(message, 0, 0) == pytest.approx(1.25)
    assert [
        is_valid_depth(v)
        for v in (1.0, 0.0, -1.0, float("nan"), float("inf"), -float("inf"))
    ] == [True, False, False, False, False, False]


def test_depth_distribution_and_patch_do_not_rewrite_published_values():
    import struct

    values = [0.125, float("inf"), 0.125, 0.25]
    message = image(
        "32FC1", width=2, height=2, step=8, data=struct.pack("<4f", *values)
    )
    distribution = depth_frame_distribution(message)
    assert (
        distribution["finite_count"] == 3
        and distribution["positive_infinity_count"] == 1
    )
    assert distribution["min"] == pytest.approx(0.125) and distribution[
        "max"
    ] == pytest.approx(0.25)
    assert distribution["fixed_samples"]["center"] == pytest.approx(0.25)
    patch = depth_patch_summary(message, 0, 0)
    assert patch["center"] == pytest.approx(0.125) and patch["finite_count"] == 3


def test_depth_decoder_honors_row_padding_and_big_endian_data():
    import struct

    payload = struct.pack(">f", 1.25) + b"pad!" + struct.pack(">f", 2.5) + b"pad!"
    message = image("32FC1", width=1, height=2, step=8, data=payload, big=True)
    decoded = decode_depth_frame(message)
    assert decoded.shape == (2, 1)
    assert depth_value_at(message, 0, 0, decoded=decoded) == pytest.approx(1.25)
    assert depth_value_at(message, 1, 0, decoded=decoded) == pytest.approx(2.5)
    assert depth_frame_distribution(message, decoded=decoded)["mean"] == pytest.approx(
        1.875
    )


def test_decoded_depth_view_is_reused_for_statistics_samples_and_patches():
    import struct

    class CountingImage:
        encoding, width, height, step, is_bigendian = "32FC1", 2, 2, 8, False

        def __init__(self):
            self._data, self.data_reads = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0), 0

        @property
        def data(self):
            self.data_reads += 1
            return self._data

    message = CountingImage()
    decoded = decode_depth_frame(message)
    reads_after_decode = message.data_reads
    depth_frame_distribution(message, decoded=decoded)
    depth_patch_summary(message, 1, 1, decoded=decoded)
    depth_value_at(message, 0, 0, decoded=decoded)
    assert reads_after_decode == 2
    assert message.data_reads == reads_after_decode


def test_spatial_distribution_and_coordinate_hypotheses_preserve_image_layout():
    import struct

    message = image(
        "32FC1",
        width=4,
        height=2,
        step=16,
        data=struct.pack(
            "<8f", 0.125, 0.125, 0.1, float("inf"), 0.125, 0.11, 0.125, float("inf")
        ),
    )
    decoded = decode_depth_frame(message)
    distribution = depth_frame_distribution(message, decoded=decoded)
    assert distribution["finite_nonmax"]["image_bbox_xyxy"] == [1, 0, 2, 1]
    grid = coarse_depth_grid(decoded, columns=2, rows=1)
    assert [cell["finite_nonmax_count"] for cell in grid["cells"]] == [1, 1]
    assert depth_coordinate_hypotheses(4, 2, 1, 0) == {
        "identity": [1, 0],
        "horizontal_flip": [2, 0],
        "vertical_flip": [1, 1],
        "both_flips": [2, 1],
    }


def test_camera_info_checks_calibration_without_requiring_image_stamp_equality():
    info = SimpleNamespace(
        header=SimpleNamespace(frame_id="camera_color_optical_frame"),
        width=1280,
        height=720,
        k=[1.0] * 9,
        p=[1.0] * 12,
        r=[1.0] * 9,
        d=[0.0] * 5,
        distortion_model="plumb_bob",
    )
    validate_camera_info(info, "camera_color_optical_frame", (1280, 720))
    info.k[0] = float("nan")
    with pytest.raises(ValueError):
        validate_camera_info(info, "camera_color_optical_frame", (1280, 720))


def test_live_verified_camera_info_contract_is_representable_by_the_validator():
    contract = acceptance_config()
    expected = contract["camera_info"]
    info = SimpleNamespace(
        header=SimpleNamespace(frame_id=expected["frame_id"]),
        width=1280,
        height=720,
        k=expected["K"],
        p=expected["P"],
        r=expected["R"],
        d=expected["D"],
        distortion_model=expected["distortion_model"],
    )
    validate_camera_info(info, expected["frame_id"], contract["expected_resolution"])
    assert contract["offered_qos"] == {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "unknown",
    }
    assert contract["depth_unit"] == "metres"
    assert contract["depth_semantics"] == "DistanceToImagePlane_optical_axis_Z"
    assert contract["camera_info_exact_stamp_verified"] is True
