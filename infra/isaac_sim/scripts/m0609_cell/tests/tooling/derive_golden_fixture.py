"""Analyze a source rosbag and derive a small RGB-D fixture without rewriting CDR."""

import argparse
import hashlib
import json
import math
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SUPPORT = Path(__file__).resolve().parents[1] / "support"
if str(SUPPORT) not in sys.path:
    sys.path.insert(0, str(SUPPORT))

from rgbd_assertions import (
    is_valid_depth,
    stamp_nanoseconds,
    validate_camera_info,
    validate_image_payload,
)


RGB = "/camera/color/image_raw"
DEPTH = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO = "/camera/color/camera_info"
CLOCK = "/clock"
STATIC_TF = "/tf_static"
REQUIRED_TOPICS = {RGB, DEPTH, CAMERA_INFO, CLOCK, STATIC_TF}
REQUIRED_EDGES = {
    ("world", "camera_link"),
    ("camera_link", "camera_color_optical_frame"),
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_capture(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {metadata.name: metadata for metadata in reader.get_all_topics_and_types()}
    missing = REQUIRED_TOPICS - set(topics)
    if missing:
        raise RuntimeError(f"source capture lacks required topics: {sorted(missing)}")
    records = defaultdict(list)
    while reader.has_next():
        topic, cdr, storage_ns = reader.read_next()
        if topic in REQUIRED_TOPICS:
            records[topic].append((cdr, storage_ns))
    return topics, records


def decode(records, topic_metadata):
    message_type = get_message(topic_metadata.type)
    return [
        (deserialize_message(cdr, message_type), cdr, storage_ns)
        for cdr, storage_ns in records
    ]


def monotonic(values, label):
    if any(after < before for before, after in zip(values, values[1:])):
        raise RuntimeError(f"{label} timestamp rollback")


def analyze_capture(uri, expected_resolution=(1280, 720)):
    topics, records = read_capture(uri)
    decoded = {
        topic: decode(records[topic], topics[topic]) for topic in REQUIRED_TOPICS
    }
    rgb = decoded[RGB]
    depth = decoded[DEPTH]
    info = decoded[CAMERA_INFO]
    clock = decoded[CLOCK]
    if not (len(rgb) == len(depth) == len(info)):
        raise RuntimeError("RGB, depth, and CameraInfo counts differ")
    for message, _, _ in rgb + depth:
        validate_image_payload(message)
    if any(message.encoding != "rgb8" for message, _, _ in rgb):
        raise RuntimeError("RGB encoding is not consistently rgb8")
    if any(message.encoding != "32FC1" for message, _, _ in depth):
        raise RuntimeError("depth encoding is not consistently 32FC1")
    image_stamps = {
        topic: [
            stamp_nanoseconds(message.header.stamp) for message, _, _ in decoded[topic]
        ]
        for topic in (RGB, DEPTH, CAMERA_INFO)
    }
    for topic, stamps in image_stamps.items():
        monotonic(stamps, topic)
        if len(stamps) != len(set(stamps)):
            raise RuntimeError(f"{topic} has duplicate acquisition timestamps")
    if image_stamps[RGB] != image_stamps[DEPTH]:
        raise RuntimeError("RGB and depth are not exact 1:1 timestamp pairs")
    for message, _, _ in info:
        validate_camera_info(message, "camera_color_optical_frame", expected_resolution)
    first_info = info[0][0]
    calibration = {
        "frame_id": first_info.header.frame_id,
        "width": first_info.width,
        "height": first_info.height,
        "distortion_model": first_info.distortion_model,
        "D": list(first_info.d),
        "R": list(first_info.r),
        "K": list(first_info.k),
        "P": list(first_info.p),
    }
    for message, _, _ in info:
        if {
            "frame_id": message.header.frame_id,
            "width": message.width,
            "height": message.height,
            "distortion_model": message.distortion_model,
            "D": list(message.d),
            "R": list(message.r),
            "K": list(message.k),
            "P": list(message.p),
        } != calibration:
            raise RuntimeError("CameraInfo calibration changes within source capture")
    clock_stamps = [stamp_nanoseconds(message.clock) for message, _, _ in clock]
    monotonic(clock_stamps, CLOCK)
    static_transforms = {}
    for message, _, _ in decoded[STATIC_TF]:
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            static_transforms[edge] = {
                "translation_m": [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                "quaternion_xyzw": [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ],
            }
    edges = set(static_transforms)
    if not REQUIRED_EDGES <= edges:
        raise RuntimeError(
            f"source capture lacks static TF edges: {sorted(REQUIRED_EDGES - edges)}"
        )
    return {
        "topics": topics,
        "records": records,
        "decoded": decoded,
        "calibration": calibration,
        "image_stamps": image_stamps[RGB],
        "camera_info_stamps": image_stamps[CAMERA_INFO],
        "clock_stamps": clock_stamps,
        "static_tf_edges": sorted(edges),
        "static_transforms": static_transforms,
        "counts": {topic: len(records[topic]) for topic in REQUIRED_TOPICS},
    }


def verify_static_tf_snapshot(analysis, snapshot):
    expected = {
        ("world", "camera_link"): snapshot["world_to_camera_link"],
        ("camera_link", "camera_color_optical_frame"): snapshot[
            "camera_link_to_optical"
        ],
    }
    for edge, expected_transform in expected.items():
        actual = analysis["static_transforms"].get(edge)
        if actual is None:
            raise RuntimeError(f"missing static TF edge: {edge}")
        for field in ("translation_m", "quaternion_xyzw"):
            if not all(
                math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)
                for a, b in zip(actual[field], expected_transform[field])
            ):
                raise RuntimeError(
                    f"static TF {edge} differs from inspector snapshot {field}"
                )


def selected_records(analysis, pair_count):
    selected_stamps = analysis["image_stamps"][:pair_count]
    if len(selected_stamps) != pair_count:
        raise RuntimeError(
            f"source capture has fewer than {pair_count} exact RGB-D pairs"
        )
    selected = {
        RGB: [],
        DEPTH: [],
        CAMERA_INFO: [],
        CLOCK: [],
        STATIC_TF: analysis["records"][STATIC_TF],
    }
    for topic in (RGB, DEPTH):
        selected[topic] = [
            record
            for record, decoded in zip(
                analysis["records"][topic], analysis["decoded"][topic]
            )
            if stamp_nanoseconds(decoded[0].header.stamp) in set(selected_stamps)
        ]
    # CameraInfo need not exactly match image stamps. Preserve samples spanning the selected image range.
    start, end = selected_stamps[0], selected_stamps[-1]
    selected[CAMERA_INFO] = [
        record
        for record, decoded in zip(
            analysis["records"][CAMERA_INFO], analysis["decoded"][CAMERA_INFO]
        )
        if start <= stamp_nanoseconds(decoded[0].header.stamp) <= end
    ]
    clock_records = list(zip(analysis["records"][CLOCK], analysis["decoded"][CLOCK]))
    selected[CLOCK] = [
        record
        for record, decoded in clock_records
        if start <= stamp_nanoseconds(decoded[0].clock) <= end
    ]
    before = [
        item for item in clock_records if stamp_nanoseconds(item[1][0].clock) < start
    ]
    after = [
        item for item in clock_records if stamp_nanoseconds(item[1][0].clock) > end
    ]
    if before:
        selected[CLOCK].insert(0, before[-1][0])
    if after:
        selected[CLOCK].append(after[0][0])
    if not selected[CAMERA_INFO] or not selected[CLOCK]:
        raise RuntimeError("selected image range lacks CameraInfo or /clock context")
    return selected, selected_stamps


def write_fixture(output_uri, analysis, selected):
    output = Path(output_uri)
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing derived fixture: {output}")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in REQUIRED_TOPICS:
        metadata = analysis["topics"][topic]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=metadata.name,
                type=metadata.type,
                serialization_format=metadata.serialization_format,
            )
        )
    merged = [
        (storage_ns, topic, cdr)
        for topic, values in selected.items()
        for cdr, storage_ns in values
    ]
    for storage_ns, topic, cdr in sorted(merged):
        writer.write(topic, cdr, storage_ns)


def repository_revision(root):
    return subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--camera-profile", required=True)
    parser.add_argument("--repository-root", required=True)
    parser.add_argument(
        "--fixture-id",
        default="isaac_rgbd_nominal_v1",
    )
    parser.add_argument("--pair-count", type=int, default=8)
    args = parser.parse_args(argv)
    analysis = analyze_capture(args.source)
    snapshot, profile = Path(args.snapshot), Path(args.camera_profile)
    snapshot_content = json.loads(snapshot.read_text())
    verify_static_tf_snapshot(analysis, snapshot_content)
    selected, selected_stamps = selected_records(analysis, args.pair_count)
    write_fixture(args.output, analysis, selected)
    source_db = next(Path(args.source).glob("*.db3"))
    manifest = {
        "schema_version": 1,
        "fixture_id": args.fixture_id,
        "status": "PARTIALLY VERIFIED",
        "source_capture": {
            "uri": "local-artifact://source-captures/" + Path(args.source).name,
            "db3_sha256": sha256(source_db),
            "source_mutated": False,
        },
        "repository_revision": repository_revision(args.repository_root),
        "provenance": {
            "camera_snapshot_path": "local-artifact://verification-inputs/"
            + snapshot.name,
            "camera_snapshot_sha256": sha256(snapshot),
            "camera_profile_path": "local-artifact://verification-inputs/"
            + profile.name,
            "camera_profile_sha256": sha256(profile),
            "stage_snapshot": snapshot_content,
        },
        "camera": analysis["calibration"],
        "topic_types": {
            topic: analysis["topics"][topic].type for topic in REQUIRED_TOPICS
        },
        "source_topic_counts": analysis["counts"],
        "selected_topic_counts": {
            topic: len(values) for topic, values in selected.items()
        },
        "static_tf_edges": analysis["static_tf_edges"],
        "static_tf_matches_inspector_snapshot": True,
        "selected_image_stamp_range_ns": [selected_stamps[0], selected_stamps[-1]],
        "depth_unit": "NOT VERIFIED",
        "depth_semantics": "NOT VERIFIED",
        "pixel_correspondence": "NOT VERIFIED",
    }
    Path(args.manifest).write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
