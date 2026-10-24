#!/usr/bin/env python3
"""Rewrite a mapping bag's per-tick lidar fragments into full-sweep messages.

LeGO-LOAM projects each message into a 360-degree range image; feeding it
CARLA's per-tick partial arcs (the estimator bag's fragments) breaks that
assumption.  This writer merges fragments into full rotations of
``--sweep-points`` points (28800 for the 16-channel rig), exactly like
``build_map.py --sweep-points``, and emits a compact bag with

  /carla/hero/lidar   sensor_msgs/msg/PointCloud2 (XYZI, full sweeps)
  /clock              rosgraph_msgs/msg/Clock (for `ros2 bag play --clock`)

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


def read_fragment_xyz(msg):
    offsets = {field.name: field.offset for field in msg.fields}
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": msg.point_step,
        }
    )
    points = np.frombuffer(msg.data, dtype=dtype)
    return np.stack([points["x"], points["y"], points["z"]], axis=1)


def read_fragment_intensity(msg):
    offsets = {field.name: field.offset for field in msg.fields}
    if "intensity" not in offsets:
        return np.zeros(len(msg.data) // msg.point_step, dtype="<f4")
    dtype = np.dtype(
        {
            "names": ["intensity"],
            "formats": ["<f4"],
            "offsets": [offsets["intensity"]],
            "itemsize": msg.point_step,
        }
    )
    return np.frombuffer(msg.data, dtype=dtype)["intensity"]


def make_pointcloud2(typestore, stamp_s, points, intensities):
    header = typestore.types["std_msgs/msg/Header"]()
    header.frame_id = "hero/lidar"
    header.stamp = typestore.types["builtin_interfaces/msg/Time"]()
    header.stamp.sec = int(stamp_s)
    header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
    msg = typestore.types["sensor_msgs/msg/PointCloud2"]()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    packed = np.zeros(len(points), dtype=np.dtype(
        {
            "names": ["x", "y", "z", "intensity"],
            "formats": ["<f4", "<f4", "<f4", "<f4"],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        }
    ))
    packed["x"] = points[:, 0]
    packed["y"] = points[:, 1]
    packed["z"] = points[:, 2]
    packed["intensity"] = intensities
    msg.fields = [
        typestore.types["sensor_msgs/msg/PointField"](
            name=name, offset=offset, datatype=7, count=1
        )
        for name, offset in LIDAR_FIELDS
    ]
    msg.data = packed.tobytes()
    return msg


def make_clock(typestore, stamp_s):
    msg = typestore.types["rosgraph_msgs/msg/Clock"]()
    msg.clock = typestore.types["builtin_interfaces/msg/Time"]()
    msg.clock.sec = int(stamp_s)
    msg.clock.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
    return msg


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
    for stamp_s, lidar_msg, clock_msg in sweeps:
        nanos = int(round(stamp_s * 1e9))
        rows.append((1, nanos, typestore.serialize_cdr(lidar_msg, "sensor_msgs/msg/PointCloud2")))
        rows.append((2, nanos, typestore.serialize_cdr(clock_msg, "rosgraph_msgs/msg/Clock")))
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
        f"  relative_file_paths: [{db_path.name}]\n"
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
    parser.add_argument("--sweep-points", type=int, default=28800,
                        help="merge fragments into sweeps of this many points "
                             "(28800 for this rig; passthrough if messages are "
                             "already full sweeps)")
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    pending = []
    pending_intensity = []
    pending_stamp = 0.0
    sweeps = []
    with Reader(args.bag) as reader:
        clocks = []
        for connection, _, raw in reader.messages():
            if connection.topic == CLOCK_TOPIC:
                msg = typestore.deserialize_cdr(raw, connection.msgtype)
                clocks.append(stamp_seconds(msg.clock))
                continue
            if connection.topic != LIDAR_TOPIC:
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            stamp = stamp_seconds(msg.header.stamp)
            if not pending:
                pending_stamp = stamp
            pending.append(read_fragment_xyz(msg))
            pending_intensity.append(read_fragment_intensity(msg))
            if sum(len(p) for p in pending) >= 0.9 * args.sweep_points:
                points = np.concatenate(pending, axis=0)
                intensities = np.concatenate(pending_intensity, axis=0)
                keep = np.isfinite(points).all(axis=1)
                sweeps.append(
                    (
                        stamp,
                        make_pointcloud2(typestore, stamp, points[keep], intensities[keep]),
                        None,
                    )
                )
                pending, pending_intensity = [], []

    if pending:
        points = np.concatenate(pending, axis=0)
        intensities = np.concatenate(pending_intensity, axis=0)
        keep = np.isfinite(points).all(axis=1)
        sweeps.append(
            (
                pending_stamp + 0.099,
                make_pointcloud2(typestore, pending_stamp + 0.099, points[keep], intensities[keep]),
                None,
            )
        )
    if not sweeps:
        raise SystemExit("no lidar messages found")

    # Attach the latest /clock value at or before each sweep stamp.
    clocks = np.asarray(clocks) if clocks else np.zeros(0)
    merged = []
    for stamp_s, lidar_msg, _ in sweeps:
        if len(clocks):
            index = int(np.searchsorted(clocks, stamp_s, side="right")) - 1
            index = max(0, index)
            clock_s = float(clocks[index])
        else:
            clock_s = stamp_s
        merged.append((stamp_s, lidar_msg, make_clock(typestore, clock_s)))

    path = write_bag(args.out, merged, typestore)
    sizes = [m[1].width for m in sweeps]
    print(
        f"SWEEP_BAG out={path} sweeps={len(sweeps)} "
        f"points_per_sweep min={min(sizes)} median={int(np.median(sizes))} max={max(sizes)} "
        f"duration_s={sweeps[-1][0] - sweeps[0][0]:.1f}"
    )


if __name__ == "__main__":
    main()
