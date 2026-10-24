#!/usr/bin/env python3
"""Scan-to-submap GICP odometry over the mapping bag (research frontend).

Replaces KISS-ICP's point-to-point frame-to-local-map registration with
small_gicp GICP scan-to-submap alignment: the current sweep is registered
against the keyframes around the current pose (60 m radius), with the
previous pose as the initial guess.  Keyframes are added on 1 m / 3 deg
motion.  No global optimization here -- feed the TUM into
``build_map.py --poses-tum --pose-graph`` for GNSS factors + loop closures.

Prints path-length/closure diagnostics comparable to ``build_map --kiss-only``.
"""
import argparse
import math
from pathlib import Path

import numpy as np

from build_map import gnss_enu_path_length, read_bag
from scipy.spatial.transform import Rotation


def write_tum(path, stamps, poses):
    lines = []
    for stamp, pose in zip(stamps, poses):
        tx, ty, tz = pose[:3, 3]
        qx, qy, qz, qw = Rotation.from_matrix(pose[:3, :3]).as_quat()
        lines.append(
            f"{stamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
            f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
        )
    Path(path).write_text("".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("out_tum")
    parser.add_argument("--voxel", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--submap-radius", type=float, default=60.0)
    parser.add_argument("--keyframe-dist", type=float, default=1.0)
    parser.add_argument("--keyframe-yaw-deg", type=float, default=3.0)
    parser.add_argument("--max-corr", type=float, default=1.5)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--num-threads", type=int, default=4)
    args = parser.parse_args()

    import small_gicp

    print("reading bag", flush=True)
    scans, scan_stamps, gnss, _ = read_bag(args.bag, 0, sweep_merge=True)
    print(f"scans={len(scans)} gnss={len(gnss)}", flush=True)

    keyframes = []  # (xyz of pose, preprocessed cloud, world points, yaw)

    def rebuild_submap(pose):
        clouds = [
            cloud
            for (xyz, cloud, _points, _yaw) in keyframes
            if np.linalg.norm(xyz - pose[:3, 3]) <= args.submap_radius
        ]
        merged = np.concatenate(clouds, axis=0)
        target, tree = small_gicp.preprocess_points(
            merged, args.voxel, 10, args.num_threads
        )
        return target, tree

    def pose_yaw(pose):
        return math.atan2(pose[1, 0], pose[0, 0])

    poses = []
    failures = 0
    for index, (scan, stamp) in enumerate(zip(scans, scan_stamps)):
        far = np.linalg.norm(scan[:, :2], axis=1) <= args.max_range
        points = scan[far]
        source, _ = small_gicp.preprocess_points(points, args.voxel, 10, args.num_threads)
        if index == 0:
            pose = np.eye(4)
            keyframes.append(
                (pose[:3, 3], source, points @ pose[:3, :3].T + pose[:3, 3], pose_yaw(pose))
            )
            poses.append(pose)
            submap = rebuild_submap(pose)
            continue
        try:
            result = small_gicp.align(
                submap[0],
                source,
                submap[1],
                poses[-1],
                "GICP",
                args.max_corr,
                args.num_threads,
                args.max_iterations,
                1e-4,
                1e-4,
            )
            pose = np.asarray(result.T_target_source, dtype=np.float64)
            if not result.converged:
                failures += 1
        except Exception as error:  # noqa: BLE001 - keep the sweep chain alive
            print(f"align failure at {index}: {error}; holding pose", flush=True)
            failures += 1
            pose = poses[-1].copy()
        poses.append(pose)

        moved = np.linalg.norm(pose[:3, 3] - keyframes[-1][0])
        turned = abs(math.atan2(
            math.sin(pose_yaw(pose) - keyframes[-1][3]),
            math.cos(pose_yaw(pose) - keyframes[-1][3]),
        ))
        if moved >= args.keyframe_dist or turned >= math.radians(args.keyframe_yaw_deg):
            keyframes.append(
                (pose[:3, 3], source, points @ pose[:3, :3].T + pose[:3, 3], pose_yaw(pose))
            )
            submap = rebuild_submap(pose)
        if index % 200 == 0:
            print(f"gicp_odom {index}/{len(scans)} keyframes={len(keyframes)}", flush=True)

    write_tum(args.out_tum, scan_stamps, poses)

    positions = np.asarray([pose[:3, 3] for pose in poses])
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    path_m = float(np.sum(steps))
    gnss_path = gnss_enu_path_length(gnss, float(scan_stamps[0]), float(scan_stamps[-1]))
    ratio = path_m / gnss_path if gnss_path else 0.0
    z = positions[:, 2]
    print(
        f"GICP_ODOM scans={len(scans)} keyframes={len(keyframes)} failures={failures} "
        f"closure_m={float(np.linalg.norm(positions[-1] - positions[0])):.2f} "
        f"closure_z={float(abs(z[-1] - z[0])):.3f} z_range=[{z.min():.2f},{z.max():.2f}] "
        f"path_m={path_m:.2f} gnss_path_m={gnss_path:.2f} ratio={ratio:.3f} "
        f"median_step_m={float(np.median(steps)):.4f}"
    )


if __name__ == "__main__":
    main()
