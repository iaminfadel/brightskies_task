#!/usr/bin/env python3
"""Absolute trajectory error of a map bundle against CARLA ground truth.

Evaluation path only (contract: GT never touches the estimator). Reads the
pseudo.odom ground truth recorded in the mapping run's *eval bag* (a separate
ros2 bag, never part of the estimator bag), maps it into the map frame with
T_map_carla from map_eval.yaml, and scores trajectory.tum against it.

Reports absolute pose error (checks anchor, yaw and T_map_carla end to end)
and Umeyama-aligned error (pure drift, offset removed).

Gates: abs RMSE <= 0.30 m, abs p95 <= 0.60 m, yaw RMSE <= 2.0 deg.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

GT_TOPIC = "/carla/hero/odometry"
GNSS_TOPIC = "/carla/hero/gnss"

GATES = {"abs_rmse_max_m": 0.30, "abs_p95_max_m": 0.60, "yaw_rmse_max_deg": 2.0}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_gt(bag_dir):
    """(stamps, xy, yaws) of hero odometry in the bridge ROS-world frame."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    stamps, xy, yaws = [], [], []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            if connection.topic != GT_TOPIC:
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            pose = msg.pose.pose
            stamps.append(stamp_seconds(msg.header.stamp))
            xy.append((pose.position.x, pose.position.y))
            yaws.append(yaw_from_quat(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w))
    if not stamps:
        raise SystemExit(f"no {GT_TOPIC} in {bag_dir}; record the eval bag")
    return np.asarray(stamps), np.asarray(xy), np.asarray(yaws)


def read_gnss_enu(bag_dir, lat0, lon0):
    """GNSS projected to the map ENU frame, to sanity-check conventions."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    radius = 6378137.0
    out = []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            if connection.topic != GNSS_TOPIC:
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            if msg.status.status < 0:
                continue
            east = math.radians(msg.longitude - lon0) * radius * math.cos(math.radians(lat0))
            north = math.radians(msg.latitude - lat0) * radius / 0.9933056
            out.append((stamp_seconds(msg.header.stamp), east, north))
    return np.asarray(out)


def read_tum(path):
    stamps, xy, yaws = [], [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        stamps.append(float(parts[0]))
        xy.append((float(parts[1]), float(parts[2])))
        yaws.append(yaw_from_quat(float(parts[5]), float(parts[6]), float(parts[7]), float(parts[4])))
    return np.asarray(stamps), np.asarray(xy), np.asarray(yaws)


def umeyama(src, dst, with_scale=False):
    """Rigid (optionally scaled) transform minimizing ||dst - (s R src + t)||."""
    mu_src, mu_dst = src.mean(axis=0), dst.mean(axis=0)
    centered_src, centered_dst = src - mu_src, dst - mu_dst
    dim = src.shape[1]
    cov = centered_dst.T @ centered_src / len(src)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(dim)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1, -1] = -1.0
    rotation = u @ s @ vt
    scale = 1.0
    if with_scale:
        var_src = (centered_src ** 2).sum() / len(src)
        scale = (cov[0, 0] + cov[1, 1] + cov[2, 2]) / var_src
    translation = mu_dst - scale * rotation @ mu_src
    return scale, rotation, translation


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def evaluate(map_dir, estimator_bag, eval_bag, args):
    map_eval = json.loads((map_dir / "map_eval.yaml").read_text())
    map_yaml = json.loads((map_dir / "map.yaml").read_text())
    t_mc = np.asarray(map_eval["T_map_carla"], dtype=np.float64).reshape(4, 4)
    lat0 = map_yaml["anchor"]["lat_deg"]
    lon0 = map_yaml["anchor"]["lon_deg"]

    stamps_gt, xy_gt, yaw_gt = read_gt(eval_bag)
    stamps_tum, xy_tum, yaw_tum = read_tum(map_dir / "trajectory.tum")

    # Bridge odometry may arrive in CARLA left-handed xy; the map ENU path from
    # the zero-noise GNSS is truth-derived, so pick the convention that agrees.
    gnss = read_gnss_enu(estimator_bag, lat0, lon0)
    if len(gnss):
        near = np.abs(gnss[:, 0, None] - stamps_gt[None, :]).argmin(axis=0)
        gn_xy = gnss[near, 1:]
        err = np.linalg.norm(xy_gt - gn_xy, axis=1).mean()
        err_flip = np.linalg.norm(xy_gt * [1, -1] - gn_xy, axis=1).mean()
        if err_flip < err:
            xy_gt = xy_gt * [1, -1]
            yaw_gt = -yaw_gt
            print(f"ATE convention: flipped odometry y (mean GNSS agreement "
                  f"{err_flip:.3f} m < {err:.3f} m)")
        else:
            print(f"ATE convention: odometry as-is (mean GNSS agreement {err:.3f} m)")

    # T_map_carla maps ROS-world xy to map xy (built from the spawn; no pitch).
    to_map = t_mc[:2, :2] @ xy_gt.T + t_mc[:2, 3:4]
    xy_gt_map = to_map.T

    idx = np.abs(stamps_gt[:, None] - stamps_tum[None, :]).argmin(axis=1)
    dt = np.abs(stamps_gt - stamps_tum[idx])
    keep = dt <= args.max_dt
    xy_gt_map, yaw_gt, xy_tum_k, yaw_tum_k = (
        xy_gt_map[keep], yaw_gt[keep], xy_tum[idx][keep], yaw_tum[idx][keep])

    errors = np.linalg.norm(xy_tum_k - xy_gt_map, axis=1)
    yaw_err = np.degrees(np.abs(wrap(yaw_tum_k - yaw_gt)))

    # Umeyama-aligned drift (removes the global rigid offset).
    _, rotation, translation = umeyama(xy_tum_k, xy_gt_map)
    aligned = (rotation @ xy_tum_k.T).T + translation
    drift = np.linalg.norm(aligned - xy_gt_map, axis=1)

    abs_rmse = float(np.sqrt(np.mean(errors ** 2)))
    abs_p95 = float(np.percentile(errors, 95))
    yaw_rmse = float(np.sqrt(np.mean(yaw_err ** 2)))
    passed = (
        abs_rmse <= GATES["abs_rmse_max_m"]
        and abs_p95 <= GATES["abs_p95_max_m"]
        and yaw_rmse <= GATES["yaw_rmse_max_deg"]
    )
    summary = {
        "schema_version": 1,
        "map_id": map_yaml.get("map_id"),
        "poses": int(keep.sum()),
        "dropped_dt": int((~keep).sum()),
        "abs_rmse_m": abs_rmse,
        "abs_p95_m": abs_p95,
        "abs_max_m": float(errors.max()),
        "aligned_rmse_m": float(np.sqrt(np.mean(drift ** 2))),
        "aligned_max_m": float(drift.max()),
        "yaw_rmse_deg": yaw_rmse,
        "yaw_p95_deg": float(np.percentile(yaw_err, 95)),
        "gates": GATES,
        "passed": bool(passed),
        "source": "carla_pseudo_odom eval bag (evaluation path only)",
    }
    (map_dir / "validation").mkdir(exist_ok=True)
    (map_dir / "validation" / "ate.json").write_text(json.dumps(summary, indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=120)
    axes[0].plot(xy_gt_map[:, 0], xy_gt_map[:, 1], label="ground truth", linewidth=2)
    axes[0].plot(xy_tum_k[:, 0], xy_tum_k[:, 1], label="trajectory.tum", linewidth=1)
    axes[0].set_aspect("equal")
    axes[0].legend()
    axes[0].set_title("map-frame trajectories")
    axes[1].plot(stamps_gt[keep], errors, label=f"abs (p95={abs_p95:.2f} m)")
    axes[1].plot(stamps_gt[keep], drift, label=f"aligned (max={drift.max():.2f} m)")
    axes[1].set_xlabel("stamp [s]")
    axes[1].set_ylabel("error [m]")
    axes[1].legend()
    axes[1].set_title(f"ATE  rmse={abs_rmse:.3f} m  yaw_rmse={yaw_rmse:.2f} deg")
    fig.tight_layout()
    fig.savefig(map_dir / "validation" / "ate.png")
    plt.close(fig)

    print(
        f"ATE_SUMMARY poses={summary['poses']} abs_rmse_m={abs_rmse:.3f} "
        f"abs_p95_m={abs_p95:.3f} aligned_rmse_m={summary['aligned_rmse_m']:.3f} "
        f"yaw_rmse_deg={yaw_rmse:.2f} gates={GATES} passed={passed}"
    )
    return passed


def self_test():
    rng = np.random.default_rng(3)
    gt = np.column_stack([np.linspace(0, 100, 200), np.sin(np.linspace(0, 6, 200)) * 10])
    theta = np.radians(2.0)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    est = (rotation @ gt.T).T + [1.5, -0.3] + rng.normal(0, 0.05, gt.shape)
    _, rot, trans = umeyama(est, gt)
    errors = np.linalg.norm((rot @ est.T).T + trans - gt, axis=1)
    # noise floor: sigma=0.05/axis in 2D -> expected aligned RMSE ~0.071
    assert float(np.sqrt(np.mean(errors ** 2))) < 0.09, "umeyama failed"
    wrap_check = wrap(np.array([3.0 * np.pi]))
    assert abs(abs(wrap_check[0]) - np.pi) < 1e-9
    print("ATE_SELF_TEST ok")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", nargs="?")
    parser.add_argument("estimator_bag", nargs="?")
    parser.add_argument("eval_bag", nargs="?")
    parser.add_argument("--max-dt", type=float, default=0.15)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.estimator_bag or not args.eval_bag:
        parser.error("need <map_dir> <estimator_bag> <eval_bag> (or --self-test)")
    if not evaluate(Path(args.map_dir), args.estimator_bag, args.eval_bag, args):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
