#!/usr/bin/env python3
"""Rewrite a mapping bag's per-tick lidar fragments into full-sweep messages.

LeGO-LOAM projects each message into a 360-degree range image; feeding it
CARLA's per-tick partial arcs (the estimator bag's fragments) breaks that
assumption.  This writer merges fragments into full rotations with the same
azimuth-span rule as ``build_map.py --sweep-merge`` and emits a compact bag
with

  /carla/hero/lidar   sensor_msgs/msg/PointCloud2 (XYZI, full sweeps)
  /clock              rosgraph_msgs/msg/Clock (for `ros2 bag play --clock`)

Intensity is zeroed: CARLA's per-point intensity carries no phase information
and the port derives relTime from it, so zero relTime matches the undistorted
per-tick CARLA snapshots.

Run inside the deps image (needs rosbags + numpy).
"""
import argparse
import sqlite3
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

LIDAR_TOPIC = "/carla/hero/lidar"
CLOCK_TOPIC = "/clock"

QOS_PROFILES = ""  # default (reliable) publisher QoS; LeGO subscribes default

LIDAR_FIELDS = (
    ("x", 0),
    ("y", 4),
    ("z", 8),
    ("intensity", 12),
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def make_pointcloud2(typestore, stamp_s, points):
    stamp = typestore.types["builtin_interfaces/msg/Time"](
        sec=int(stamp_s), nanosec=int(round((stamp_s - int(stamp_s)) * 1e9))
    )
    header = typestore.types["std_msgs/msg/Header"](stamp=stamp, frame_id="hero/lidar")
    fields = [
        typestore.types["sensor_msgs/msg/PointField"](
            name=name, offset=offset, datatype=7, count=1
        )
        for name, offset in LIDAR_FIELDS
    ]
    packed = np.zeros(
        len(points),
        dtype=np.dtype(
            {
                "names": ["x", "y", "z", "intensity"],
                "formats": ["<f4", "<f4", "<f4", "<f4"],
                "offsets": [0, 4, 8, 12],
                "itemsize": 16,
            }
        ),
    )
    packed["x"] = points[:, 0]
    packed["y"] = points[:, 1]
    packed["z"] = points[:, 2]
    msg = typestore.types["sensor_msgs/msg/PointCloud2"](
        header=header,
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points),
        data=packed.view(np.uint8),
        is_dense=True,
    )
    return msg


def make_clock(typestore, stamp_s):
    clock = typestore.types["builtin_interfaces/msg/Time"](
        sec=int(stamp_s), nanosec=int(round((stamp_s - int(stamp_s)) * 1e9))
    )
    return typestore.types["rosgraph_msgs/msg/Clock"](clock=clock)


def write_bag(out_dir, sweeps, typestore):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    db_path = out_dir / f"{out_dir.name}_0.db3"
    conn = sqlite3.connect(db_path)
    conn.executescript(
        """
        CREATE TABLE topics (
          id INTEGER PRIMARY KEY,
          name TEXT NOT NULL,
          type TEXT NOT NULL,
          serialization_format TEXT NOT NULL,
          offered_qos_profiles TEXT NOT NULL,
          type_description_hash TEXT NOT NULL);
        CREATE TABLE messages (
          id INTEGER PRIMARY KEY,
          topic_id INTEGER NOT NULL,
          timestamp INTEGER NOT NULL,
          data BLOB NOT NULL);
        CREATE INDEX timestamp_idx ON messages (timestamp);
        """
    )
    topics = [
        (1, LIDAR_TOPIC, "sensor_msgs/msg/PointCloud2", "cdr", QOS_PROFILES, ""),
        (2, CLOCK_TOPIC, "rosgraph_msgs/msg/Clock", "cdr", QOS_PROFILES, ""),
    ]
    conn.executemany("INSERT INTO topics VALUES (?,?,?,?,?,?)", topics)
    rows = []
    for stamp_s, lidar_msg in sweeps:
        nanos = int(round(stamp_s * 1e9))
        rows.append((1, nanos, typestore.serialize_cdr(lidar_msg, "sensor_msgs/msg/PointCloud2")))
        rows.append((2, nanos, typestore.serialize_cdr(make_clock(typestore, stamp_s), "rosgraph_msgs/msg/Clock")))
    conn.executemany("INSERT INTO messages (topic_id, timestamp, data) VALUES (?,?,?)", rows)
    conn.commit()
    conn.close()

    starting = int(round(sweeps[0][0] * 1e9))
    ending = int(round(sweeps[-1][0] * 1e9))
    lidar_count = len(sweeps)
    metadata = (
        "rosbag2_bagfile_information:\n"
        "  version: 5\n"
        "  storage_identifier: sqlite3\n"
        "  relative_file_paths:\n"
        f"    - {db_path.name}\n"
        "  files:\n"
        f"    - path: {db_path.name}\n"
        "      starting_time:\n"
        f"        nanoseconds_since_epoch: {starting}\n"
        "      duration:\n"
        f"        nanoseconds: {ending - starting}\n"
        f"      message_count: {2 * lidar_count}\n"
        "  compression: ''\n"
        f"  duration:\n    nanoseconds: {ending - starting}\n"
        f"  starting_time:\n    nanoseconds_since_epoch: {starting}\n"
        f"  message_count: {2 * lidar_count}\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata:\n"
        f"        name: {LIDAR_TOPIC}\n"
        "        type: sensor_msgs/msg/PointCloud2\n"
        "        serialization_format: cdr\n"
        "        offered_qos_profiles: ''\n"
        f"      message_count: {lidar_count}\n"
        "    - topic_metadata:\n"
        f"        name: {CLOCK_TOPIC}\n"
        "        type: rosgraph_msgs/msg/Clock\n"
        "        serialization_format: cdr\n"
        "        offered_qos_profiles: ''\n"
        f"      message_count: {lidar_count}\n"
        "  compression_format: ''\n"
        "  compression_mode: ''\n"
    )
    (out_dir / "metadata.yaml").write_text(metadata, encoding="utf-8")
    return db_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("out")
    args = parser.parse_args()

    from build_map import read_bag

    scans, stamps, _, _ = read_bag(args.bag, 0, sweep_merge=True)
    if not scans:
        raise SystemExit("no lidar messages found")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    sweeps = [
        (float(stamp), make_pointcloud2(typestore, float(stamp), points))
        for points, stamp in zip(scans, stamps)
    ]
    path = write_bag(args.out, sweeps, typestore)
    sizes = [len(points) for points in scans]
    print(
        f"SWEEP_BAG out={path} sweeps={len(sweeps)} "
        f"points_per_sweep min={min(sizes)} median={int(np.median(sizes))} max={max(sizes)} "
        f"duration_s={sweeps[-1][0] - sweeps[0][0]:.1f}"
    )


if __name__ == "__main__":
    main()
