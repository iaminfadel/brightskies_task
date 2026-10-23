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
  validation/heldout/scans   held-out scans (map frame) and poses.csv
  validation/heldout.yaml     held-out selection and GICP settings
  validation/coverage.json    coverage and map statistics
  validation/overlay.png      map plus trajectory overlay
  manifest.json               bundle file list and hashes

No CARLA ground-truth topics are read.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from kiss_icp.kiss_icp import KissICP, KISSConfig
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

LIDAR_TOPIC = "/carla/hero/lidar"
GNSS_TOPIC = "/carla/hero/gnss"
IMU_TOPIC = "/carla/hero/imu"


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def repo_git_sha():
    for env_name, fallback in (
        (os.environ.get("BRIGHTDRIVE_GIT_SHA"), None),
        (None, "/ws/brightdrive/.deployed-sha"),
    ):
        candidate = env_name or fallback
        if not candidate:
            continue
        if env_name:
            return env_name.strip()
        try:
            value = Path(candidate).read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if value:
            return value
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).resolve().parents[1],
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def bag_sha256(bag_path):
    root = Path(bag_path)
    if root.is_file():
        return sha256_file(root)
    db3 = sorted(root.glob("*.db3"))
    if not db3:
        return "unknown"
    digest = hashlib.sha256()
    for path in db3:
        digest.update(f"{path.name}:{sha256_file(path)}".encode())
    return digest.hexdigest()


def package_version(name):
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


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
                if msg.status.status < 0:  # STATUS_NO_FIX (-1); keep SBAS/GBAS fixes
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


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_priors(gnss, imu, scan_stamps, lever_m):
    """LiDAR yaw and position for every scan from GNSS/IMU samples."""
    yaws = np.asarray([nearest_imu(imu, stamp)[1] for stamp in scan_stamps])
    positions = np.zeros((len(scan_stamps), 3))
    lat0 = float(gnss[0, 1])
    lon0 = float(gnss[0, 2])
    radius = 6378137.0
    gnss_index = np.searchsorted(gnss[:, 0], scan_stamps)
    gnss_index = np.clip(gnss_index, 0, len(gnss) - 1)
    for i, (stamp, j) in enumerate(zip(scan_stamps, gnss_index)):
        if j > 0 and abs(gnss[j - 1, 0] - stamp) < abs(gnss[j, 0] - stamp):
            j -= 1
        east = math.radians(gnss[j, 2] - lon0) * radius * math.cos(
            math.radians(lat0)
        )
        north = (
            math.radians(gnss[j, 1] - lat0) * radius / 0.9933056
        )
        positions[i, 0] = east - lever_m * math.cos(yaws[i])
        positions[i, 1] = north - lever_m * math.sin(yaws[i])
        positions[i, 2] = float(gnss[j, 3]) - float(gnss[0, 3]) + 0.4
    return yaws, positions


def gnss_course_priors(gnss, imu, scan_stamps, lever_m):
    """LiDAR yaw from GNSS course over ground and GNSS-derived positions."""
    radius = 6378137.0
    lat0 = float(gnss[0, 1])
    lon0 = float(gnss[0, 2])
    east = np.radians(gnss[:, 2] - lon0) * radius * math.cos(math.radians(lat0))
    north = np.radians(gnss[:, 1] - lat0) * radius / 0.9933056
    count = len(gnss)
    courses = np.zeros(count)
    # While the vehicle is stationary the GNSS course is undefined; seed with
    # the IMU yaw and keep it until motion produces a valid course.
    previous = float(nearest_imu(imu, scan_stamps[0])[1])
    locked = False
    for j in range(count):
        de = dn = 0.0
        for window in (2, 5, 10, 20):
            a = max(0, j - window)
            b = min(count - 1, j + window)
            de = float(east[b] - east[a])
            dn = float(north[b] - north[a])
            if de * de + dn * dn >= 0.04:
                break
        if de * de + dn * dn >= 0.01:
            candidate = math.atan2(dn, de)
            if not locked:
                previous = candidate
                locked = True
            elif abs(wrap_angle(candidate - previous)) < math.radians(20.0):
                previous = previous + wrap_angle(candidate - previous)
        courses[j] = previous
    indices = np.searchsorted(gnss[:, 0], scan_stamps)
    indices = np.clip(indices, 0, count - 1)
    for i, (stamp, j) in enumerate(zip(scan_stamps, indices)):
        if j > 0 and abs(gnss[j - 1, 0] - stamp) < abs(gnss[j, 0] - stamp):
            indices[i] = j - 1
    yaws = courses[indices]
    positions = np.zeros((len(scan_stamps), 3))
    for i, j in enumerate(indices):
        positions[i, 0] = east[j] - lever_m * math.cos(yaws[i])
        positions[i, 1] = north[j] - lever_m * math.sin(yaws[i])
        positions[i, 2] = float(gnss[j, 3]) - float(gnss[0, 3]) + 0.4
    return yaws, positions


def run_kiss(
    scans,
    scan_stamps,
    imu,
    gnss,
    base_lidar,
    base_gnss,
    voxel_size,
    max_range,
    min_motion_th=0.1,
    fixed_threshold=None,
    imu_delta=False,
    gnss_delta=False,
):
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
    yaws = positions = None
    if imu_delta or gnss_delta:
        yaws, positions = scan_priors(
            gnss, imu, scan_stamps, float(base_gnss[0]) - float(base_lidar[0])
        )
    poses = []
    timestamps = np.zeros(1)
    for index, scan in enumerate(scans):
        if index > 0 and (imu_delta or gnss_delta):
            delta = kiss.last_delta.copy()
            dyaw = wrap_angle(yaws[index] - yaws[index - 1])
            cosine, sine = math.cos(dyaw), math.sin(dyaw)
            delta[:3, :3] = np.array(
                [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
            )
            if gnss_delta:
                step = positions[index] - positions[index - 1]
                cosine, sine = math.cos(yaws[index - 1]), math.sin(yaws[index - 1])
                delta[:3, 3] = [
                    cosine * step[0] + sine * step[1],
                    -sine * step[0] + cosine * step[1],
                    step[2],
                ]
            kiss.last_delta = delta
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
    parser.add_argument("--imu-delta", action="store_true")
    parser.add_argument("--gnss-delta", action="store_true")
    parser.add_argument("--gnss-pose", action="store_true")
    parser.add_argument("--gnss-yaw", action="store_true")
    parser.add_argument("--pose-graph", action="store_true",
                        help="KISS odometry + GNSS factors + ICP loop closures, "
                             "globally optimized (offline SLAM)")
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "validation" / "heldout" / "scans").mkdir(parents=True, exist_ok=True)

    spawn = [float(value) for value in args.spawn.split(",")]
    base_lidar = [float(value) for value in args.base_lidar.split(",")]
    base_gnss = [float(value) for value in args.base_gnss.split(",")]

    print("reading bag", flush=True)
    scans, scan_stamps, gnss, imu = read_bag(args.bag, args.max_scans)
    print(f"scans={len(scans)} gnss={len(gnss)} imu={len(imu)}", flush=True)

    print("running kiss-icp", flush=True)
    slam_poses = run_kiss(
        scans,
        scan_stamps,
        imu,
        gnss,
        base_lidar,
        base_gnss,
        args.kiss_voxel,
        args.max_range,
        args.min_motion_th,
        args.fixed_threshold,
        args.imu_delta,
        args.gnss_delta,
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
            f"fixed_threshold={args.fixed_threshold} imu_delta={args.imu_delta} "
            f"gnss_delta={args.gnss_delta} "
            f"closure_m={float(np.linalg.norm(positions[-1] - positions[0])):.2f} "
            f"closure_z={float(abs(slam_poses[-1][2, 3] - slam_poses[0][2, 3])):.3f} "
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

    if args.pose_graph:
        import pose_graph as pg

        print("pose graph: initial states from KISS + anchor", flush=True)
        initial = np.asarray(
            [pg.matrix_to_state(T_map_slam @ pose) for pose in slam_poses]
        )
        odom_edges = [
            {"i": k, "j": k + 1, "m": pg.relative(initial[k], initial[k + 1])}
            for k in range(len(initial) - 1)
        ]
        lever = float(base_gnss[0]) - float(base_lidar[0])
        _, gnss_positions = scan_priors(gnss, imu, scan_stamps, lever)
        gnss_edges = [
            {"i": k, "p": gnss_positions[k, :3].tolist()}
            for k in range(0, len(initial), 3)
        ]
        print(f"pose graph: {len(odom_edges)} odom, {len(gnss_edges)} gnss edges; "
              f"detecting loops", flush=True)
        loop_edges = pg.build_loop_edges(scans, initial, scan_stamps)
        print(f"pose graph: {len(loop_edges)} loop edges; optimizing", flush=True)
        optimized = pg.optimize(initial, odom_edges, gnss_edges, loop_edges)
        slam_poses = np.asarray(
            [np.linalg.inv(T_map_slam) @ pg.state_to_matrix(state) for state in optimized]
        )

    if args.gnss_pose:
        lever = float(base_gnss[0]) - float(base_lidar[0])
        if args.gnss_yaw:
            yaws, positions = gnss_course_priors(gnss, imu, scan_stamps, lever)
        else:
            _, positions = scan_priors(gnss, imu, scan_stamps, lever)
        map_poses = []
        for pose, position, yaw in zip(
            slam_poses, positions, yaws if args.gnss_yaw else [None] * len(positions)
        ):
            map_pose = np.eye(4)
            if args.gnss_yaw:
                map_pose[:3, :3] = from_xyzyaw(0.0, 0.0, 0.0, float(yaw))[:3, :3]
            else:
                map_pose[:3, :3] = T_map_slam[:3, :3] @ pose[:3, :3]
            map_pose[:3, 3] = position
            map_poses.append(map_pose)
        slam_poses = np.asarray(
            [np.linalg.inv(T_map_slam) @ pose for pose in map_poses]
        )

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

    heldout_dir = out_dir / "validation" / "heldout"
    heldout_poses = []
    heldout_points = []
    for index in sorted(heldout_indices):
        pose = T_map_lidar[index]
        rotation = pose[:3, :3]
        points = (rotation @ scans[index].T).T + pose[:3, 3]
        scan_path = heldout_dir / "scans" / f"scan_{index:04d}.pcd"
        write_pcd(scan_path, points)
        heldout_points.append(points)
        heldout_poses.append(
            {
                "index": int(index),
                "stamp": float(scan_stamps[index]),
                "file": str(scan_path.relative_to(out_dir)),
                "T_map_lidar": pose.reshape(-1).tolist(),
            }
        )

    if heldout_points:
        tree = cKDTree(cloud)
        per_scan_residual = [
            float(np.median(tree.query(points, k=1)[0])) for points in heldout_points
        ]
        heldout_residual_m = float(np.median(per_scan_residual))
    else:
        heldout_residual_m = 0.0

    def pose_row(item):
        pose = np.asarray(item["T_map_lidar"], dtype=np.float64).reshape(4, 4)
        quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
        x, y, z = pose[:3, 3]
        return [
            item["index"],
            f"{item['stamp']:.9f}",
            f"{x:.6f}",
            f"{y:.6f}",
            f"{z:.6f}",
            f"{quat[0]:.9f}",
            f"{quat[1]:.9f}",
            f"{quat[2]:.9f}",
            f"{quat[3]:.9f}",
        ]

    with (heldout_dir / "poses.csv").open("w", encoding="utf-8") as handle:
        handle.write("index,stamp,x,y,z,qx,qy,qz,qw\n")
        for item in heldout_poses:
            handle.write(",".join(str(value) for value in pose_row(item)) + "\n")

    first_stamp = float(scan_stamps[0])
    last_stamp = float(scan_stamps[-1])
    trajectory_length = float(
        np.sum(np.linalg.norm(np.diff(T_map_base_poses[:, :3, 3], axis=0), axis=1))
    )
    created_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    kiss_version = package_version("kiss-icp")
    git_sha = repo_git_sha()
    source_bag_sha256 = bag_sha256(args.bag)
    generation_command = shlex.join([sys.executable] + sys.argv)
    map_config = {
        "voxel": float(args.voxel),
        "kiss_voxel": float(args.kiss_voxel),
        "max_range": float(args.max_range),
        "coverage_radius": float(args.coverage_radius),
        "gnss_pose": bool(args.gnss_pose),
        "gnss_yaw": bool(args.gnss_yaw),
        "imu_delta": bool(args.imu_delta),
        "gnss_delta": bool(args.gnss_delta),
        "kiss_icp_version": kiss_version,
    }
    config_sha256 = hashlib.sha256(
        json.dumps(map_config, sort_keys=True).encode()
    ).hexdigest()
    map_yaml = {
        "schema_version": 1,
        "map_id": args.map_id,
        "map_frame": "map",
        "created_at": created_at,
        "anchor": {
            "lat_deg": float(anchor_lat),
            "lon_deg": float(anchor_lon),
            "alt_m": float(anchor_alt),
            "source": "first_gnss_fix",
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
            "bag_sha256": source_bag_sha256,
            "git_sha": git_sha,
            "kiss_icp_version": kiss_version,
            "config_sha256": config_sha256,
            "scan_count": scan_count,
            "heldout_count": len(heldout_indices),
            "first_stamp": first_stamp,
            "last_stamp": last_stamp,
            "trajectory_length_m": trajectory_length,
            "pose_source": (
                "gnss_positions_course_yaw"
                if (args.gnss_pose and args.gnss_yaw)
                else "pose_graph_kiss_gnss_loops"
                if args.pose_graph
                else "gnss_imu_scan_rotation"
                if args.gnss_pose
                else "kiss_icp"
            ),
            "built_at": created_at,
        },
        "generation_command": generation_command,
    }
    (out_dir / "map.yaml").write_text(json.dumps(map_yaml, indent=2), encoding="utf-8")

    gicp_settings = {
        "voxel_m": 0.25,
        "coarse_correspondence_m": 1.0,
        "fine_correspondence_m": 0.5,
        "max_iterations": 30,
        "convergence_eps": 1e-4,
        "num_threads": 2,
        "num_neighbors": 10,
        "crop_m": 80.0,
    }
    gicp_sha = hashlib.sha256(
        json.dumps(gicp_settings, sort_keys=True).encode()
    ).hexdigest()
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
        "T_map_carla_validation": {
            "method": "heldout_scan_alignment",
            "samples": len(heldout_indices),
            "residual_m": round(heldout_residual_m, 6),
            "threshold_m": 0.5,
            "note": "median nearest-map-point distance of held-out scans at their recorded poses; mapping bag is sensor-only",
        },
        "heldout": {
            "count": len(heldout_indices),
            "selection": "same_bag_arc_length",
            "directory": "validation/heldout",
            "gicp_settings_sha256": gicp_sha,
        },
        "source_run_id": out_dir.name,
    }
    (out_dir / "map_eval.yaml").write_text(json.dumps(map_eval, indent=2), encoding="utf-8")

    heldout_yaml = {
        "schema_version": 1,
        "map_id": args.map_id,
        "count": len(heldout_indices),
        "selection": "same_bag_arc_length",
        "directory": "validation/heldout",
        "first_stamp": first_stamp,
        "last_stamp": last_stamp,
        "scan_indices": sorted(int(index) for index in heldout_indices),
        "excluded_from_accumulation": True,
        "gicp": gicp_settings,
        "gicp_settings_sha256": gicp_sha,
        "source_run_id": out_dir.name,
    }
    (out_dir / "validation" / "heldout.yaml").write_text(
        json.dumps(heldout_yaml, indent=2), encoding="utf-8"
    )

    coverage = {
        "map_id": args.map_id,
        "point_count": point_count,
        "trajectory_length_m": trajectory_length,
        "coverage_radius_m": float(args.coverage_radius),
        "bounds": {"min": bounds_min, "max": bounds_max},
        "scan_count": scan_count,
        "heldout_count": len(heldout_indices),
        "mean_scan_points": float(np.mean([len(scan) for scan in scans])),
        "scenarios": {},
        "note": "per-scenario coverage is filled once scenario routes are pinned",
    }
    (out_dir / "validation" / "coverage.json").write_text(
        json.dumps(coverage, indent=2), encoding="utf-8"
    )

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    trajectory_xy = T_map_base_poses[:, :2, 3]
    stride = max(1, len(cloud) // 400000)
    shown = cloud[::stride]
    fig, ax = plt.subplots(figsize=(12, 12), dpi=120)
    ax.scatter(shown[:, 0], shown[:, 1], s=0.05, c=shown[:, 2],
               cmap="viridis", linewidths=0, rasterized=True)
    ax.plot(trajectory_xy[:, 0], trajectory_xy[:, 1], color="tab:red", linewidth=1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{args.map_id}: {point_count} points, {trajectory_length:.0f} m")
    fig.tight_layout()
    fig.savefig(out_dir / "validation" / "overlay.png")
    plt.close(fig)

    manifest = {
        "schema_version": 1,
        "map_id": args.map_id,
        "created_at": created_at,
        "pcd_sha256": pcd_sha256,
        "files": [],
        "source_bag_sha256": source_bag_sha256,
        "git_sha": git_sha,
        "tool_versions": {
            "kiss_icp": kiss_version,
            "small_gicp": package_version("small-gicp"),
            "rosbags": package_version("rosbags"),
        },
        "generation_command": generation_command,
    }
    digests = {}
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            relative = str(path.relative_to(out_dir))
            digest = sha256_file(path)
            digests[relative] = digest
            manifest["files"].append(
                {"path": relative, "bytes": path.stat().st_size, "sha256": digest}
            )
    bundle = hashlib.sha256(
        "".join(f"{name}:{digest}" for name, digest in sorted(digests.items())).encode()
    ).hexdigest()
    manifest["bundle_sha256"] = bundle
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"map written: {out_dir} points={point_count} pcd_sha256={pcd_sha256}")
    print(f"trajectory_length_m={trajectory_length:.1f} bundle_sha256={bundle}")


if __name__ == "__main__":
    main()
