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
  tiles/             tile_{ix}_{iy}.pcd, the map split on a regular 2D X/Y grid
  map_index.yaml     tile grid origin, size, Z range and the tile file list
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
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

LIDAR_TOPIC = "/carla/hero/lidar"
GNSS_TOPIC = "/carla/hero/gnss"
IMU_TOPIC = "/carla/hero/imu"

# One map cell per tile; 25 m is a few tiles across the 50 m lidar window, so a
# windowed map_server loads in single digits (ADR 0003).
TILE_SIZE_M = 25.0


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


def read_bag(bag_dir, max_scans=0, sweep_merge=False):
    """Read lidar/gnss/imu. With sweep_merge, per-tick lidar fragments (CARLA
    publishes a partial arc each tick) are merged into full rotations: a
    sweep closes when the pending points cover the full azimuth circle
    (largest angular gap small) -- coverage, not counted angle, because
    fragment arcs are ~70% of a rotation and overlap. The merged stamp is the
    last fragment's stamp."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    scans, gnss, imu = [], [], []
    scan_stamps = []
    pending = []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            topic = connection.topic
            if topic not in (LIDAR_TOPIC, GNSS_TOPIC, IMU_TOPIC):
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            stamp = stamp_seconds(msg.header.stamp)
            if topic == LIDAR_TOPIC:
                points = pointcloud2_xyz(msg)
                if sweep_merge:
                    pending.append(points)
                    merged = np.concatenate(pending, axis=0)
                    azimuths = np.sort(np.arctan2(merged[:, 1], merged[:, 0]))
                    gaps = np.diff(azimuths)
                    wrap_gap = 2.0 * math.pi - (azimuths[-1] - azimuths[0])
                    largest_gap = max(float(gaps.max()), wrap_gap) if len(gaps) else math.pi
                    if largest_gap <= 0.44:
                        scans.append(merged)
                        scan_stamps.append(stamp)
                        pending = []
                else:
                    scans.append(points)
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
    return np.arctan2(np.sin(angle), np.cos(angle))


def gnss_enu(gnss, scan_stamps):
    """Antenna ENU (east, north, altitude-delta) per scan, anchored at the
    first fix."""
    lat0 = float(gnss[0, 1])
    lon0 = float(gnss[0, 2])
    radius = 6378137.0
    gnss_index = np.searchsorted(gnss[:, 0], scan_stamps)
    gnss_index = np.clip(gnss_index, 0, len(gnss) - 1)
    east = np.zeros(len(scan_stamps))
    north = np.zeros(len(scan_stamps))
    alt = np.zeros(len(scan_stamps))
    for i, (stamp, j) in enumerate(zip(scan_stamps, gnss_index)):
        if j > 0 and abs(gnss[j - 1, 0] - stamp) < abs(gnss[j, 0] - stamp):
            j -= 1
        # CARLA's GNSS sensor is plain equirectangular about the georeference
        # origin (verified: world->ENU is identity+translation for this map);
        # no extra scale factor.
        east[i] = math.radians(gnss[j, 2] - lon0) * radius * math.cos(math.radians(lat0))
        north[i] = math.radians(gnss[j, 1] - lat0) * radius
        alt[i] = float(gnss[j, 3]) - float(gnss[0, 3])
    return east, north, alt


def gnss_imu_states(gnss, imu, scan_stamps, lever_m):
    """Trajectory states from GNSS positions + IMU yaw in the map ENU frame.
    The IMU reports CARLA-frame heading; sign and constant offset are
    resolved against the GNSS course over moving windows."""
    yaws_raw = np.asarray([nearest_imu(imu, stamp)[1] for stamp in scan_stamps])
    course_yaws, _ = gnss_course_priors(gnss, imu, scan_stamps, lever_m)
    moving = np.abs(wrap_angle(course_yaws - np.roll(course_yaws, 20))) > math.radians(5)
    moving[:20] = False

    def disagreement(sign, offset):
        return np.abs(
            wrap_angle(sign * yaws_raw[moving] + offset - course_yaws[moving])
        ).mean()

    sign, offset = min(
        (
            (s, wrap_angle(np.median(course_yaws[moving] - s * yaws_raw[moving])))
            for s in (1.0, -1.0)
        ),
        key=lambda so: disagreement(*so),
    )
    print(
        f"gnss_imu_states: imu yaw sign {sign:+.0f}, offset "
        f"{math.degrees(offset):.1f} deg, course disagreement "
        f"{math.degrees(disagreement(sign, offset)):.2f} deg",
        flush=True,
    )
    yaws = np.asarray([wrap_angle(sign * yaw + offset) for yaw in yaws_raw])
    east, north, alt = gnss_enu(gnss, scan_stamps)
    positions = np.column_stack(
        [
            east - lever_m * np.cos(yaws),
            north - lever_m * np.sin(yaws),
            alt + 0.4,
        ]
    )
    return np.column_stack([positions[:, 0], positions[:, 1], yaws, positions[:, 2]])


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
            math.radians(gnss[j, 1] - lat0) * radius
        )
        positions[i, 0] = east - lever_m * math.cos(yaws[i])
        positions[i, 1] = north - lever_m * math.sin(yaws[i])
        positions[i, 2] = float(gnss[j, 3]) - float(gnss[0, 3]) + 0.4
    return yaws, positions


def gnss_course_priors(gnss, imu, scan_stamps):
    """LiDAR yaw priors from the GNSS course over ground (valid only while
    actually moving; stationary stretches hold the last course)."""
    radius = 6378137.0
    lat0 = float(gnss[0, 1])
    lon0 = float(gnss[0, 2])
    east = np.radians(gnss[:, 2] - lon0) * radius * math.cos(math.radians(lat0))
    north = np.radians(gnss[:, 1] - lat0) * radius
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
    return courses[indices]


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
):
    from kiss_icp.kiss_icp import KissICP, KISSConfig

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


def tile_cells(points, origin_xy, tile_size):
    """O(1) floor division: the grid cell of every point's x/y."""
    return np.floor((points[:, :2] - origin_xy) / tile_size).astype(np.int64)


def tile_lookup(index, x, y):
    """O(1) floor division against the index origin and tile size; the contract
    the map_server consumes (ADR 0003)."""
    return (
        math.floor((x - float(index["origin"]["x_m"])) / float(index["tile_size_m"])),
        math.floor((y - float(index["origin"]["y_m"])) / float(index["tile_size_m"])),
    )


def write_tiles(cloud, out_dir, map_id, tile_size):
    """Split the already voxel-downsampled cloud into a regular 2D X/Y grid of
    tiles (full Z range, so a windowed consumer needs no second pass). Additive:
    the full map is written before this and is untouched by it."""
    tiles_dir = out_dir / "tiles"
    tiles_dir.mkdir(parents=True, exist_ok=True)
    # A degenerate map (everything filtered out) gets an empty grid, not a crash.
    origin_xy = np.asarray([cloud[:, 0].min(), cloud[:, 1].min()]) if len(cloud) else np.zeros(2)
    z_range = [float(cloud[:, 2].min()), float(cloud[:, 2].max())] if len(cloud) else [0.0, 0.0]
    cells = tile_cells(cloud, origin_xy, tile_size)
    index = {
        "schema_version": 1,
        "map_id": map_id,
        "map_frame": "map",
        "tile_size_m": float(tile_size),
        "origin": {"x_m": float(origin_xy[0]), "y_m": float(origin_xy[1])},
        "z_range_m": z_range,
        "tile_count": 0,
        "point_count": int(len(cloud)),
        "files": [],
    }
    written = 0
    for ix, iy in sorted({(int(a), int(b)) for a, b in cells}):
        points = cloud[(cells[:, 0] == ix) & (cells[:, 1] == iy)]
        path = tiles_dir / f"tile_{ix}_{iy}.pcd"
        write_pcd(path, points)
        index["files"].append(
            {
                "path": str(path.relative_to(out_dir)),
                "ix": ix,
                "iy": iy,
                "point_count": int(len(points)),
            }
        )
        written += 1
    index["tile_count"] = written
    (out_dir / "map_index.yaml").write_text(json.dumps(index, indent=2), encoding="utf-8")
    return index


def self_check():
    """Tile lookup and the tile split, checked against hand-computed values."""
    origin = np.array([-2.5, 0.5])
    size = 2.5
    points = np.array(
        [
            [-2.5, 0.5],    # the origin itself -> (0, 0)
            [0.0, 3.0],     # inside cell (1, 1)
            [0.0, 3.0 - 1e-9],  # just below the cell edge -> (1, 0)
            [-5.0, -2.0],   # negative cells -> (-1, -1)
            [2.4999999, 5.4999999],  # upper-right corner of (1, 1)
        ]
    )
    expected = np.array([[0, 0], [1, 1], [1, 0], [-1, -1], [1, 1]])
    cells = tile_cells(points, origin, size)
    assert (cells == expected).all(), f"tile_cells wrong: {cells.tolist()} != {expected.tolist()}"
    index = {"origin": {"x_m": origin[0], "y_m": origin[1]}, "tile_size_m": size}
    # Every point sits inside the half-open cell the lookup named, the origin
    # corner belongs to cell (0, 0), and the vectorized tile_cells and the
    # scalar tile_lookup are two shapes of one contract -- so they cannot drift.
    lower = origin + cells * size
    upper = lower + size
    assert (points >= lower).all() and (points < upper).all(), "lookup outside its cell"
    assert all(
        tuple(cells[i]) == tile_lookup(index, *points[i]) for i in range(len(points))
    ), "tile_cells and tile_lookup disagree"

    # The split: no point lost, none duplicated, and every tile holds only the
    # points its own lookup names.
    rng = np.random.default_rng(0)
    cloud = rng.uniform(-10.0, 10.0, size=(5000, 3))
    with tempfile.TemporaryDirectory() as tmp:
        index = write_tiles(cloud, Path(tmp), "self-check", 4.0)
        origin = np.array([index["origin"]["x_m"], index["origin"]["y_m"]])
        size = index["tile_size_m"]
        reloaded = []
        for entry in index["files"]:
            ix, iy = entry["ix"], entry["iy"]
            path = Path(tmp) / entry["path"]
            header, _, body = path.read_bytes().partition(b"DATA binary\n")
            count = int(next(
                line.split()[1] for line in header.decode("ascii").splitlines()
                if line.startswith("POINTS")
            ))
            assert count == entry["point_count"], f"{entry['path']} point count changed"
            points = np.frombuffer(body, dtype="<f4", count=count * 3).reshape(-1, 3)
            # Points are stored as float32, so a point sitting on a cell edge can
            # round into the neighbour; contain it up to that storage noise
            # instead of demanding exact equality (the lookup itself is exact).
            slack = 1e-6 * max(1.0, abs(ix) * size, abs(iy) * size)
            lower = origin + np.array([ix, iy]) * size - slack
            upper = origin + np.array([ix + 1, iy + 1]) * size + slack
            assert (points[:, :2] >= lower).all() and (points[:, :2] < upper).all(), \
                f"{entry['path']} holds points from another cell"
            reloaded.append(points)
        merged = np.concatenate(reloaded, axis=0)
        assert len(merged) == len(cloud), "tiles lost or duplicated points"
        assert np.allclose(np.sort(merged[:, 0]), np.sort(cloud[:, 0]), atol=1e-5), \
            "tile points changed"
    print(f"TILE_SELF_CHECK ok=true tiles={index['tile_count']} points={index['point_count']}")


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
    parser.add_argument("bag", nargs="?")
    parser.add_argument("out", nargs="?")
    parser.add_argument("--map-id")
    parser.add_argument("--spawn", help="x,y,z,yaw in CARLA world")
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--kiss-voxel", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--coverage-radius", type=float, default=20.0)
    parser.add_argument("--heldout", type=int, default=20)
    parser.add_argument("--base-lidar", default="1.394,0.0,2.700")
    parser.add_argument("--base-gnss", default="2.394,0.0,2.300")
    parser.add_argument("--base-hero", default="1.394,0.0,0.300",
                        help="hero origin in base_link (URDF)")
    parser.add_argument("--outlier-k", type=int, default=16)
    parser.add_argument("--outlier-sigma", type=float, default=1.0)
    parser.add_argument("--min-motion-th", type=float, default=0.1)
    parser.add_argument("--fixed-threshold", type=float, default=None)
    parser.add_argument("--max-scans", type=int, default=0)
    parser.add_argument("--gnss-imu", action="store_true",
                        help="diagnostic baseline: pose the map from GNSS "
                             "antenna positions + IMU yaw (no lidar); passes "
                             "gates only because CARLA's GNSS/IMU are "
                             "noise-free -- never the shipping path")
    parser.add_argument("--pose-graph", action="store_true",
                        help="KISS odometry + GNSS factors + GNSS-vetoed ICP "
                             "loop closures, globally optimized (offline "
                             "SLAM); the default and canonical path")
    parser.add_argument("--pg-dump", default="",
                        help="pose-graph mode: dump graph inputs to this .npz (debug)")
    parser.add_argument("--pg-load", default="",
                        help="pose-graph mode: load graph inputs from this .npz "
                             "and skip KISS + loop detection (same --sweep-merge)")
    parser.add_argument("--pg-optimized", default="",
                        help="load pre-solved optimized states (.npz key "
                             "'optimized') and skip the graph entirely")
    parser.add_argument("--pg-redo-loops", action="store_true",
                        help="with --pg-load: re-detect loop edges from the bag "
                              "scans at the loaded states")
    parser.add_argument("--loop-radius", type=float, default=6.0,
                        help="max distance between revisited scan positions "
                        "(tight enough to exclude parallel streets)")
    parser.add_argument("--sweep-merge", action="store_true",
                        help="merge per-tick lidar fragments into full rotations "
                        "by azimuth span (required for KISS on async bags)")
    parser.add_argument("--tile-size", type=float, default=TILE_SIZE_M,
                        help="tile edge length in metres for the tiles/ grid")
    parser.add_argument("--no-tiles", action="store_true",
                        help="write the full map only (no tiles/ + map_index.yaml)")
    parser.add_argument("--self-check", action="store_true",
                        help="run the tile lookup/split self-check and exit")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not (args.bag and args.out and args.map_id and args.spawn):
        parser.error("need <bag> <out> --map-id <id> --spawn x,y,z,yaw (or --self-check)")
    if args.tile_size <= 0.0:
        parser.error("--tile-size must be positive")
    # Canonical default: the lidar pose graph.
    if not (args.gnss_imu or args.pose_graph or args.pg_load or args.pg_optimized):
        args.pose_graph = True

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "validation" / "heldout" / "scans").mkdir(parents=True, exist_ok=True)

    spawn = [float(value) for value in args.spawn.split(",")]
    base_lidar = [float(value) for value in args.base_lidar.split(",")]
    base_gnss = [float(value) for value in args.base_gnss.split(",")]
    base_hero = [float(value) for value in args.base_hero.split(",")]

    print("reading bag", flush=True)
    scans, scan_stamps, gnss, imu = read_bag(args.bag, args.max_scans, args.sweep_merge)
    print(f"scans={len(scans)} gnss={len(gnss)} imu={len(imu)}", flush=True)

    if not args.pg_load and not args.pg_optimized and not args.gnss_imu:
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
        )

    anchor_stamp, anchor_lat, anchor_lon, anchor_alt = gnss[0]
    _, anchor_yaw = nearest_imu(imu, anchor_stamp)
    T_base_gnss = translation(*base_gnss)
    T_base_lidar = translation(*base_lidar)
    T_map_gnss = from_xyzyaw(0.0, 0.0, 0.0, anchor_yaw)
    T_map_base = T_map_gnss @ np.linalg.inv(T_base_gnss)
    T_map_slam = T_map_base @ T_base_lidar

    if args.gnss_imu:
        # CARLA's GNSS is an exact antenna position and the IMU yaw is the
        # body yaw; together they give a drift-free pose for every scan.  This
        # is the deterministic map trajectory: no KISS drift, no loop edges.
        lever = float(base_gnss[0]) - float(base_lidar[0])
        yaws, positions = scan_priors(gnss, imu, scan_stamps, lever)
        map_poses = np.asarray([
            from_xyzyaw(float(position[0]), float(position[1]), float(position[2]),
                        float(yaw))
            for position, yaw in zip(positions, yaws)
        ])
        slam_poses = np.asarray(
            [np.linalg.inv(T_map_slam) @ pose for pose in map_poses]
        )
    elif args.pg_optimized:
        # States already solved offline (preemption-resilient workflow): skip
        # the graph entirely and go straight to accumulation.
        import pose_graph as pg

        data = np.load(args.pg_optimized)
        optimized = data["optimized"]
        print(f"pose graph: loaded {len(optimized)} optimized states "
              f"from {args.pg_optimized}", flush=True)
        slam_poses = np.asarray(
            [np.linalg.inv(T_map_slam) @ pg.state_to_matrix(state) for state in optimized]
        )
    elif args.pose_graph or args.pg_load:
        import pose_graph as pg

        if args.pg_load:
            data = np.load(args.pg_load)
            initial = data["initial"]
            odom_edges = [
                {"i": int(i), "j": int(j), "m": m}
                for i, j, m in zip(data["odom_i"], data["odom_j"], data["odom_m"])
            ]
            gnss_edges = [
                {"i": int(i), "p": p}
                for i, p in zip(data["gnss_i"], data["gnss_p"])
            ]
            # The dump stores antenna positions only; the lever and course-yaw
            # keys are rebuilt from the bag (the graph models the lever
            # in-graph -- dropping it constrains lidar poses to antenna
            # positions, a systematic 1 m heading-direction error).
            lever = float(base_gnss[0]) - float(base_lidar[0])
            course, _ = gnss_course_priors(gnss, imu, scan_stamps, lever)
            moving = np.abs(wrap_angle(course - np.roll(course, 15))) > math.radians(2.0)
            for edge in gnss_edges:
                edge["lever"] = lever
                if moving[edge["i"]]:
                    edge["yaw"] = float(course[edge["i"]])
            loop_edges = [
                {"i": int(i), "j": int(j), "m": m}
                for i, j, m in zip(data["loop_i"], data["loop_j"], data["loop_m"])
            ]
            if len(initial) != len(scans):
                raise SystemExit(
                    f"pg_load has {len(initial)} states for {len(scans)} scans; "
                    "use the same --sweep-merge"
                )
            print(f"pose graph: loaded {len(initial)} states, "
                  f"{len(loop_edges)} loop edges from {args.pg_load}", flush=True)
        else:
            print("pose graph: initial states from KISS + anchor", flush=True)
            initial = np.asarray(
                [pg.matrix_to_state(T_map_slam @ pose) for pose in slam_poses]
            )
            odom_edges = [
                {"i": k, "j": k + 1, "m": pg.relative(initial[k], initial[k + 1])}
                for k in range(len(initial) - 1)
            ]
            lever = float(base_gnss[0]) - float(base_lidar[0])
            east, north, alt = gnss_enu(gnss, scan_stamps)
            course, _ = gnss_course_priors(gnss, imu, scan_stamps, lever)
            # GNSS factors: antenna ENU positions with the antenna->lidar
            # lever modeled in-graph (rotates with the state yaw), plus
            # course-over-ground yaw where the course is actually moving
            # (stale stationary values carry no information). The initial
            # yaw convention is irrelevant -- the optimizer recovers the
            # frame from these factors alone.
            moving = np.abs(wrap_angle(course - np.roll(course, 15))) > math.radians(2.0)
            gnss_edges = []
            for k in range(0, len(initial), 3):
                edge = {"i": k, "p": [east[k], north[k], alt + 0.4], "lever": lever}
                if moving[k]:
                    edge["yaw"] = float(course[k])
                gnss_edges.append(edge)
            print(f"pose graph: {sum('yaw' in e for e in gnss_edges)} yaw factors, "
                  f"lever {lever:.2f} m", flush=True)
        if not args.pg_load or args.pg_redo_loops:
            if args.pg_load:
                print("pose graph: re-detecting loops from loaded states", flush=True)
            else:
                print(f"pose graph: {len(odom_edges)} odom, {len(gnss_edges)} gnss edges; "
                      f"detecting loops", flush=True)
            loop_edges = pg.build_loop_edges(
                scans, initial, scan_stamps, radius=args.loop_radius
            )
        # Vet ICP loop edges against the GNSS-implied relative poses before
        # they enter the graph: mis-locks on repeated geometry otherwise pull
        # the solution and hide their own residual.
        gnss_reference = gnss_imu_states(
            gnss, imu, scan_stamps, float(base_gnss[0]) - float(base_lidar[0])
        )
        loop_edges = pg.vet_loop_edges(loop_edges, gnss_reference)
        print(f"pose graph: {len(loop_edges)} loop edges; optimizing", flush=True)
        if args.pg_dump:
            np.savez(
                args.pg_dump,
                initial=initial,
                stamps=scan_stamps,
                odom_i=[e["i"] for e in odom_edges],
                odom_j=[e["j"] for e in odom_edges],
                odom_m=np.asarray([e["m"] for e in odom_edges]),
                gnss_i=[e["i"] for e in gnss_edges],
                gnss_p=np.asarray([e["p"] for e in gnss_edges]),
                loop_i=[e["i"] for e in loop_edges],
                loop_j=[e["j"] for e in loop_edges],
                loop_m=np.asarray([e["m"] for e in loop_edges]),
            )
        optimized = pg.optimize(initial, odom_edges, gnss_edges, loop_edges)
        slam_poses = np.asarray(
            [np.linalg.inv(T_map_slam) @ pg.state_to_matrix(state) for state in optimized]
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

    tile_index = None
    if not args.no_tiles:
        tile_index = write_tiles(cloud, out_dir, args.map_id, args.tile_size)
        print(f"tiles written: {tile_index['tile_count']} "
              f"({args.tile_size:g} m grid)", flush=True)

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
        "sweep_merge": bool(args.sweep_merge),
        "gnss_imu": bool(args.gnss_imu),
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
                "gnss_positions_imu_yaw"
                if args.gnss_imu
                else "pose_graph_kiss_gnss_loops"
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
    T_base_hero = translation(*base_hero)
    T_ros_hero = from_xyzyaw(spawn_x, -spawn_y, spawn_z, -math.radians(spawn_yaw))
    T_ros_base = T_ros_hero @ np.linalg.inv(T_base_hero)
    T_ros_slam = T_ros_hero @ T_base_lidar
    # The spawn is a CARLA hero pose; evaluators need a rigid base_link->base_link
    # transform, so subtract the hero->base lever before mapping.
    T_map_carla = T_map_base @ np.linalg.inv(T_ros_base)
    map_eval = {
        "schema_version": 1,
        "map_id": args.map_id,
        "transform_semantics": "ros_base_link_to_map_base_link",
        "T_map_carla": T_map_carla.reshape(-1).tolist(),
        "T_carla_slam": T_ros_slam.reshape(-1).tolist(),
        "base_hero": list(base_hero),
        "base_lidar": list(base_lidar),
        "base_gnss": list(base_gnss),
        "spawn_carla": {
            "x": spawn_x,
            "y": spawn_y,
            "z": spawn_z,
            "yaw_deg": spawn_yaw,
        },
        "note": "T_map_carla maps ROS-world base_link poses (x east, y north, yaw ENU) to the map base_link frame.",
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
