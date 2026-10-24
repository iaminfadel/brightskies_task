#!/usr/bin/env python3
"""Extract per-sweep lidar poses from a recorded LeGO-LOAM run bag.

Reads the LeGO output topics from the recorded run bag, decodes LeGO-LOAM's
camera-frame odometry conventions (the original ROS1 publishTF quaternion
remix, which the Humble port inherits), converts them to ROS-lidar-frame
4x4 poses with the origin at the first lidar pose, interpolates to the
sweep stamps of the mapping bag, and writes a TUM file for
``build_map.py --poses-tum``.

The port publishes translation in LeGO's internal camera axes
(x left, y up, z forward).  Two decodes are computed:

  a  original ROS1 remix: camera frame, P-permuted to lidar axes
  b  plain: message position/quaternion taken as lidar-frame already

``--decode auto`` picks the candidate whose z range is smaller (CARLA roads
are near-planar, so a z range of kilometres means the forward axis leaked
into height).  Both candidates' diagnostics are always printed.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

ODOM_CANDIDATES = [
    "/integrated_to_init",  # transformFusion: per-association fused output
    "/aft_mapped_to_init",  # mapOptimization: per-mapping-cycle optimized
]

# camera (x_l = z_c, y_l = x_c, z_l = y_c)
AXIS_P = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def read_odometry(bag_dir, topic):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    rows = []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            if connection.topic != topic:
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            pose = msg.pose.pose
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rows.append(
                (
                    stamp,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                )
            )
    return np.asarray(rows, dtype=np.float64)


def decode_candidate(rows, mode):
    """(stamps, 4x4 poses) in the lidar frame with origin at the first pose."""
    stamps = rows[:, 0]
    translations = rows[:, 1:4]
    quats = rows[:, 4:8]
    poses = []
    if mode == "b":
        rotations = Rotation.from_quat(quats)
        for t, quat, position in zip(stamps, rotations, translations):
            pose = np.eye(4)
            pose[:3, :3] = quat.as_matrix()
            pose[:3, 3] = position
            poses.append(pose)
    else:
        for t, quat, position in zip(stamps, quats, translations):
            # invert the original publishTF remix:
            # msg.quat = (-g.y, -g.z, g.x, g.w) with g = RPY(yaw, -roll, -pitch)
            q_tf = np.array([quat[3], quat[2], -quat[0], -quat[1]])  # w, x, y, z
            roll_i, pitch_i, yaw_i = Rotation.from_quat(q_tf[[1, 2, 3, 0]]).as_euler("xyz")
            # original laserOdometryHandler inverse
            transform_roll = -pitch_i
            transform_pitch = -yaw_i
            transform_yaw = roll_i
            rot_cam = (
                Rotation.from_euler("z", transform_yaw)
                * Rotation.from_euler("y", transform_pitch)
                * Rotation.from_euler("x", transform_roll)
            ).as_matrix()
            rot_lidar = AXIS_P @ rot_cam @ AXIS_P.T
            pos_lidar = AXIS_P @ position
            pose = np.eye(4)
            pose[:3, :3] = rot_lidar
            pose[:3, 3] = pos_lidar
            poses.append(pose)
    if not poses:
        raise SystemExit(f"no odometry rows to decode (mode {mode})")
    poses = np.asarray(poses)
    poses = np.asarray([np.linalg.inv(poses[0]) @ pose for pose in poses])
    return stamps, poses


def interpolate_to(stamps, poses, targets):
    """Held-endpoint SE(3) interpolation of the pose chain at target stamps."""
    keep = np.concatenate([[True], np.diff(stamps) > 1e-6])
    stamps = stamps[keep]
    poses = poses[keep]
    if len(stamps) == 1:
        return np.repeat(poses, len(targets), axis=0)
    rotations = Slerp(stamps, Rotation.from_matrix(poses[:, :3, :3]))
    interp_rot = rotations(np.clip(targets, stamps[0], stamps[-1])).as_matrix()
    interp_pos = np.column_stack(
        [np.interp(targets, stamps, poses[:, k, 3]) for k in range(3)]
    )
    out = np.repeat(np.eye(4)[None, :, :], len(targets), axis=0)
    out[:, :3, :3] = interp_rot
    out[:, :3, 3] = interp_pos
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("lego_bag", help="bag recorded during the LeGO run")
    parser.add_argument("mapping_bag", help="estimator bag the sweeps came from")
    parser.add_argument("out_tum")
    parser.add_argument("--decode", choices=["auto", "a", "b"], default="auto")
    parser.add_argument("--topic", default="",
                        help="override odometry topic (default: first present of "
                             + ", ".join(ODOM_CANDIDATES))
    args = parser.parse_args()

    topic = args.topic
    if not topic:
        with Reader(args.lego_bag) as reader:
            present = {connection.topic for connection in reader.connections}
        for candidate in ODOM_CANDIDATES:
            if candidate in present:
                topic = candidate
                break
        if not topic:
            raise SystemExit(f"none of {ODOM_CANDIDATES} in {args.lego_bag}; "
                             f"topics: {sorted(present)}")
    rows = read_odometry(args.lego_bag, topic)
    if not len(rows):
        raise SystemExit(f"topic {topic} has no messages")
    print(f"LEGO_ODOM topic={topic} messages={len(rows)}")

    from build_map import read_bag

    scans, sweep_stamps, _, _ = read_bag(args.mapping_bag, 0, sweep_merge=True)
    print(f"mapping bag: {len(scans)} sweeps")

    decoded = {}
    for mode in ("a", "b"):
        stamps, poses = decode_candidate(rows, mode)
        interp = interpolate_to(stamps, poses, sweep_stamps)
        decoded[mode] = interp
        z = interp[:, 2, 3]
        path = float(np.sum(np.linalg.norm(np.diff(interp[:, :2, 3], axis=0), axis=1)))
        print(
            f"decode_{mode}: z_range=[{z.min():.2f},{z.max():.2f}] "
            f"closure={np.linalg.norm(interp[-1, :3, 3] - interp[0, :3, 3]):.2f} m "
            f"path={path:.1f} m"
        )

    if args.decode == "auto":
        z_a = np.ptp(decoded["a"][:, 2, 3])
        z_b = np.ptp(decoded["b"][:, 2, 3])
        mode = "a" if z_a <= z_b else "b"
        print(f"decode_auto: z_a={z_a:.2f} z_b={z_b:.2f} -> {mode}")
    else:
        mode = args.decode
    poses = decoded[mode]

    lines = []
    for stamp, pose in zip(sweep_stamps, poses):
        tx, ty, tz = pose[:3, 3]
        qx, qy, qz, qw = Rotation.from_matrix(pose[:3, :3]).as_quat()
        lines.append(
            f"{stamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
            f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
        )
    Path(args.out_tum).write_text("".join(lines), encoding="utf-8")
    print(f"LEGO_TUM out={args.out_tum} poses={len(poses)} decode={mode} topic={topic}")


if __name__ == "__main__":
    main()
