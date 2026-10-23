#!/usr/bin/env python3
"""Build a localization map from a sensor-only CARLA mapping bag.

The bag must contain ``/carla/hero/lidar`` (sensor_msgs/PointCloud2),
``/carla/hero/gnss`` (NavSatFix) and ``/carla/hero/imu`` (Imu) recorded in
simulation time.  Streams the raw registered LiDAR scans through KISS-ICP,
anchors the SLAM trajectory at the first GNSS fix with the IMU yaw and the
known base->sensor extrinsics, and writes the map bundle consumed by
``brightdrive_localization``:

  map.pcd            registered, downsampled point cloud in the map (ENU) frame
  map.yaml           anchor, T_map_slam, checksum and provenance
  map_eval.yaml      evaluator-only T_map_carla and spawn metadata
  trajectory.tum     base_link poses in the map frame
  heldout/           held-out scans (map frame) and their poses
  coverage.json      coverage and map statistics
  manifest.json      bundle file list and hashes

No CARLA ground-truth topics are read.
"""
import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from kiss_icp.kiss_icp import KissICP, KISSConfig
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

LIDAR_TOPIC = "/carla/hero/lidar"
GNSS_TOPIC = "/carla/hero/gnss"
IMU_TOPIC = "/carla/hero/imu"


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pointcloud2_xyz(msg):
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
    xyz = np.stack([points["x"], points["y"], points["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def from_xyzyaw(x, y, z, yaw):
    transform = np.eye(4)
    transform[:3, :3] = [
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[:3, 3] = [x, y, z]
    return transform


def translation(x, y, z):
    transform = np.eye(4)
    transform[:3, 3] = [x, y, z]
    return transform


def quaternion_from_matrix(matrix):
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return x, y, z, w


def read_bag(bag_dir, max_scans=0):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    scans, gnss, imu = [], [], []
    scan_stamps = []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            topic = connection.topic
            if topic not in (LIDAR_TOPIC, GNSS_TOPIC, IMU_TOPIC):
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            stamp = stamp_seconds(msg.header.stamp)
            if topic == LIDAR_TOPIC:
                scans.append(pointcloud2_xyz(msg))
                scan_stamps.append(stamp)
                if max_scans and len(scans) >= max_scans:
                    break
            elif topic == GNSS_TOPIC:
                if msg.status.status == 2:  # STATUS_NO_FIX
                    continue
                gnss.append((stamp, msg.latitude, msg.longitude, msg.altitude))
            else:
                orientation = msg.orientation
                imu.append(
                    (
                        stamp,
                        yaw_from_quaternion(
                            orientation.x, orientation.y, orientation.z, orientation.w
                        ),
                    )
                )
    if not scans or not gnss or not imu:
        raise SystemExit("bag is missing LiDAR, GNSS or IMU messages")
    return scans, np.asarray(scan_stamps), np.asarray(gnss), np.asarray(imu)


def gnss_enu_path_length(gnss, stamp_min, stamp_max):
    window = gnss[(gnss[:, 0] >= stamp_min) & (gnss[:, 0] <= stamp_max)]
    if len(window) < 2:
        return 0.0
    radius = 6378137.0
    east = np.radians(window[:, 2] - window[0, 2]) * radius * math.cos(
        math.radians(window[0, 1])
    )
    north = np.radians(window[:, 1] - window[0, 1]) * radius
    steps = np.linalg.norm(np.diff(np.stack([east, north], axis=1), axis=0), axis=1)
    return float(np.sum(steps))


def nearest_imu(imu, stamp):
    index = int(np.argmin(np.abs(imu[:, 0] - stamp)))
    return imu[index]


def run_kiss(scans, voxel_size, max_range, min_motion_th=0.1, fixed_threshold=None):
    config = KISSConfig()
    config.data.max_range = float(max_range)
    config.data.min_range = 0.0
    config.data.deskew = False
    config.mapping.voxel_size = float(voxel_size)
    config.mapping.max_points_per_voxel = 20
    config.registration.max_num_iterations = 100
    config.registration.convergence_criterion = 1e-4
    config.adaptive_threshold.initial_threshold = 2.0
    config.adaptive_threshold.min_motion_th = float(min_motion_th)
    if fixed_threshold is not None:
        config.adaptive_threshold.fixed_threshold = float(fixed_threshold)
    kiss = KissICP(config)
    poses = []
    timestamps = np.zeros(1)
    for scan in scans:
        kiss.register_frame(scan, timestamps)
        poses.append(kiss.last_pose.copy())
    return np.asarray(poses)


def voxel_downsample(points, voxel_size):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique = np.unique(keys, axis=0, return_index=True)
    return points[unique]


def outlier_filter(points, k=16, sigma=1.0):
    if len(points) < k + 1:
        return points
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k + 1, workers=-1)
    kth = distances[:, -1]
    threshold = kth.mean() + sigma * kth.std()
    return points[kth < threshold]


def crop_to_trajectory(points, trajectory_xy, radius):
    if len(points) == 0 or len(trajectory_xy) == 0:
        return points
    tree = cKDTree(trajectory_xy)
    distances, _ = tree.query(points[:, :2], workers=-1)
    return points[distances <= radius]


def write_pcd(path, points):
    points = np.asarray(points, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(points.tobytes())


def write_tum(path, stamps, poses):
    lines = []
    for stamp, pose in zip(stamps, poses):
        tx, ty, tz = pose[:3, 3]
        qx, qy, qz, qw = quaternion_from_matrix(pose)
        lines.append(
            f"{stamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
            f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
        )
    Path(path).write_text("".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--spawn", required=True, help="x,y,z,yaw in CARLA world")
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--kiss-voxel", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--coverage-radius", type=float, default=20.0)
    parser.add_argument("--heldout", type=int, default=20)
    parser.add_argument("--base-lidar", default="1.394,0.0,2.700")
    parser.add_argument("--base-gnss", default="2.394,0.0,2.300")
    parser.add_argument("--outlier-k", type=int, default=16)
    parser.add_argument("--outlier-sigma", type=float, default=1.0)
    parser.add_argument("--min-motion-th", type=float, default=0.1)
    parser.add_argument("--fixed-threshold", type=float, default=None)
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--kiss-only", action="store_true")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "heldout" / "scans").mkdir(parents=True, exist_ok=True)

    spawn = [float(value) for value in args.spawn.split(",")]
    base_lidar = [float(value) for value in args.base_lidar.split(",")]
    base_gnss = [float(value) for value in args.base_gnss.split(",")]

    print("reading bag", flush=True)
    scans, scan_stamps, gnss, imu = read_bag(args.bag, args.max_scans)
    print(f"scans={len(scans)} gnss={len(gnss)} imu={len(imu)}", flush=True)

    print("running kiss-icp", flush=True)
    slam_poses = run_kiss(
        scans,
        args.kiss_voxel,
        args.max_range,
        args.min_motion_th,
        args.fixed_threshold,
    )
    if args.kiss_only:
        positions = np.asarray([pose[:3, 3] for pose in slam_poses])
        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        kiss_path = float(np.sum(steps))
        gnss_path = gnss_enu_path_length(
            gnss, float(scan_stamps[0]), float(scan_stamps[-1])
        )
        ratio = kiss_path / gnss_path if gnss_path else 0.0
        print(
            f"KISS_ONLY scans={len(scans)} min_motion_th={args.min_motion_th} "
            f"fixed_threshold={args.fixed_threshold} "
            f"kiss_path_m={kiss_path:.2f} gnss_path_m={gnss_path:.2f} "
            f"ratio={ratio:.3f} median_step_m={float(np.median(steps)):.4f}",
            flush=True,
        )
        return

    anchor_stamp, anchor_lat, anchor_lon, anchor_alt = gnss[0]
    _, anchor_yaw = nearest_imu(imu, anchor_stamp)
    T_base_gnss = translation(*base_gnss)
    T_base_lidar = translation(*base_lidar)
    T_map_gnss = from_xyzyaw(0.0, 0.0, 0.0, anchor_yaw)
    T_map_base = T_map_gnss @ np.linalg.inv(T_base_gnss)
    T_map_slam = T_map_base @ T_base_lidar

    T_map_lidar = np.asarray([T_map_slam @ pose for pose in slam_poses])
    T_map_base_poses = np.asarray([pose @ np.linalg.inv(T_base_lidar) for pose in T_map_lidar])

    scan_count = len(scans)
    heldout_count = min(args.heldout, max(0, scan_count // 4))
    heldout_indices = set()
    if heldout_count:
        heldout_indices = set(
            np.linspace(5, scan_count - 1, heldout_count, dtype=int).tolist()
        )
    print(f"heldout={sorted(heldout_indices)}", flush=True)

    print("accumulating map", flush=True)
    chunks = []
    for index, (scan, pose) in enumerate(zip(scans, T_map_lidar)):
        if index in heldout_indices:
            continue
        rotation = pose[:3, :3]
        points = (rotation @ scan.T).T + pose[:3, 3]
        chunks.append(points)
    cloud = np.concatenate(chunks, axis=0)
    cloud = voxel_downsample(cloud, args.voxel)
    print(f"voxel points={len(cloud)}", flush=True)
    cloud = outlier_filter(cloud, args.outlier_k, args.outlier_sigma)
    print(f"outlier points={len(cloud)}", flush=True)
    cloud = crop_to_trajectory(cloud, T_map_base_poses[:, :2, 3], args.coverage_radius)
    print(f"cropped points={len(cloud)}", flush=True)

    pcd_path = out_dir / "map.pcd"
    write_pcd(pcd_path, cloud)
    pcd_sha256 = sha256_file(pcd_path)
    point_count = int(len(cloud))
    bounds_min = cloud.min(axis=0).tolist() if point_count else [0.0, 0.0, 0.0]
    bounds_max = cloud.max(axis=0).tolist() if point_count else [0.0, 0.0, 0.0]

    write_tum(out_dir / "trajectory.tum", scan_stamps, T_map_base_poses)

    heldout_dir = out_dir / "heldout"
    heldout_poses = []
    for index in sorted(heldout_indices):
        pose = T_map_lidar[index]
        rotation = pose[:3, :3]
        points = (rotation @ scans[index].T).T + pose[:3, 3]
        scan_path = heldout_dir / "scans" / f"scan_{index:04d}.pcd"
        write_pcd(scan_path, points)
        heldout_poses.append(
            {
                "index": int(index),
                "stamp": float(scan_stamps[index]),
                "file": str(scan_path.relative_to(out_dir)),
                "T_map_lidar": pose.reshape(-1).tolist(),
            }
        )
    (heldout_dir / "poses.json").write_text(
        json.dumps(heldout_poses, indent=2), encoding="utf-8"
    )

    first_stamp = float(scan_stamps[0])
    last_stamp = float(scan_stamps[-1])
    trajectory_length = float(
        np.sum(np.linalg.norm(np.diff(T_map_base_poses[:, :3, 3], axis=0), axis=1))
    )
    map_yaml = {
        "schema_version": 1,
        "map_id": args.map_id,
        "map_frame": "map",
        "anchor": {
            "lat_deg": float(anchor_lat),
            "lon_deg": float(anchor_lon),
            "alt_m": float(anchor_alt),
            "stamp": float(anchor_stamp),
        },
        "T_map_slam": T_map_slam.reshape(-1).tolist(),
        "pcd_sha256": pcd_sha256,
        "voxel_size_m": float(args.voxel),
        "point_count": point_count,
        "coverage_radius_m": float(args.coverage_radius),
        "bounds": {"min": bounds_min, "max": bounds_max},
        "source": {
            "run_id": out_dir.name,
            "bag": str(Path(args.bag).name),
            "scan_count": scan_count,
            "heldout_count": len(heldout_indices),
            "first_stamp": first_stamp,
            "last_stamp": last_stamp,
            "trajectory_length_m": trajectory_length,
            "built_at": datetime.now(timezone.utc).isoformat(),
        },
    }
    (out_dir / "map.yaml").write_text(json.dumps(map_yaml, indent=2), encoding="utf-8")

    spawn_x, spawn_y, spawn_z, spawn_yaw = spawn[:4]
    T_ros_base = from_xyzyaw(spawn_x, -spawn_y, spawn_z, -math.radians(spawn_yaw))
    T_ros_slam = T_ros_base @ T_base_lidar
    T_map_carla = T_map_slam @ np.linalg.inv(T_ros_slam)
    map_eval = {
        "schema_version": 1,
        "map_id": args.map_id,
        "T_map_carla": T_map_carla.reshape(-1).tolist(),
        "T_carla_slam": T_ros_slam.reshape(-1).tolist(),
        "spawn_carla": {
            "x": spawn_x,
            "y": spawn_y,
            "z": spawn_z,
            "yaw_deg": spawn_yaw,
        },
        "note": "T_map_carla maps ROS-world (x east, y north, yaw ENU) poses to map.",
    }
    (out_dir / "map_eval.yaml").write_text(json.dumps(map_eval, indent=2), encoding="utf-8")

    coverage = {
        "map_id": args.map_id,
        "point_count": point_count,
        "trajectory_length_m": trajectory_length,
        "coverage_radius_m": float(args.coverage_radius),
        "bounds": {"min": bounds_min, "max": bounds_max},
        "scan_count": scan_count,
        "heldout_count": len(heldout_indices),
        "mean_scan_points": float(np.mean([len(scan) for scan in scans])),
    }
    (out_dir / "coverage.json").write_text(json.dumps(coverage, indent=2), encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "map_id": args.map_id,
        "files": {},
    }
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            manifest["files"][str(path.relative_to(out_dir))] = sha256_file(path)
    bundle = hashlib.sha256(
        "".join(f"{name}:{digest}" for name, digest in sorted(manifest["files"].items())).encode()
    ).hexdigest()
    manifest["bundle_sha256"] = bundle
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"map written: {out_dir} points={point_count} pcd_sha256={pcd_sha256}")
    print(f"trajectory_length_m={trajectory_length:.1f} bundle_sha256={bundle}")


if __name__ == "__main__":
    main()
