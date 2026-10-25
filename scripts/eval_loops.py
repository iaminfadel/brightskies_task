#!/usr/bin/env python3
"""Loop-consistency check of a map bundle against its mapping bag.

For every revisited place (scan pairs far apart in time, close in recorded
position), holds each visit's scan AT its recorded pose and scores it against
the other visit's submap by nearest-neighbour distance (bidirectional max).
That residual is the inter-visit misalignment frozen into the map -- exactly
the doubled-wall defect the held-out residual gate cannot see (it compares a
scan against its own visit's geometry). Search-free on purpose: a walked ICP
correction was tried first, but on corridor geometry its convergence basin is
so shallow that centimetre-level pose differences flip sub-metre locks into
multi-metre slides, reporting ICP basin fragility as map error (the walked
correction is still computed and kept as a diagnostic field).

Gates: >= min_pairs valid pairs, median residual <= 0.15 m, p95 <= 0.40 m.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

LIDAR_TOPIC = "/carla/hero/lidar"
GATES = {"min_pairs": 10, "median_max_m": 0.15, "p95_max_m": 0.40}


def read_tum_poses(path):
    stamps, poses = [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        stamps.append(float(parts[0]))
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(
            [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
        ).as_matrix()
        pose[:3, 3] = [float(parts[1]), float(parts[2]), float(parts[3])]
        poses.append(pose)
    return np.asarray(stamps), poses


def icp_planar(source, target_tree, init, max_corr, iterations=30):
    """Planar (yaw + xy) point-to-point ICP; returns (4x4, inlier_ratio).

    The map is accumulated from yaw-only vehicle poses with GNSS altitude.
    Full 3D point-to-point ICP is unconstrained vertically on tall facades:
    ground points can slide onto wall points while every correspondence stays
    within max_corr. Estimating only yaw and xy keeps the gate measuring the
    horizontal doubled-wall defect it targets.
    """
    transform = init.copy()
    prev_t = None
    for _ in range(iterations):
        moved = source @ transform[:3, :3].T + transform[:3, 3]
        distances, indices = target_tree.query(moved, k=1, workers=-1)
        mask = distances <= max_corr
        if mask.sum() < 20:
            break
        src_corr = moved[mask][:, :2]
        dst_corr = target_tree.data[indices[mask]][:, :2]
        mu_s, mu_d = src_corr.mean(axis=0), dst_corr.mean(axis=0)
        src_centered = src_corr - mu_s
        dst_centered = dst_corr - mu_d
        cross = float(np.sum(
            src_centered[:, 0] * dst_centered[:, 1]
            - src_centered[:, 1] * dst_centered[:, 0]
        ))
        dot = float(np.sum(
            src_centered[:, 0] * dst_centered[:, 0]
            + src_centered[:, 1] * dst_centered[:, 1]
        ))
        yaw = math.atan2(cross, dot)
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        delta = np.eye(4)
        delta[:2, :2] = rotation
        delta[:2, 3] = mu_d - rotation @ mu_s
        transform = delta @ transform
        if prev_t is not None and np.linalg.norm(transform[:3, 3] - prev_t) < 1e-4:
            break
        prev_t = transform[:3, 3].copy()
    moved = source @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = target_tree.query(moved, k=1, workers=-1)
    return transform, float(np.mean(distances <= max_corr))


def find_revisit_pairs(stamps, xy, radius_m, min_dt_s, yaws=None, max_dyaw_deg=45.0):
    """Greedy revisit pairs: close in space, similar heading, far apart in time."""
    tree = cKDTree(xy)
    candidates = []
    n = len(xy)
    for i in range(n):
        for j in tree.query_ball_point(xy[i], radius_m):
            if not (j - i > 50 and stamps[j] - stamps[i] >= min_dt_s):
                continue
            if yaws is not None:
                dyaw = abs(math.atan2(
                    math.sin(yaws[j] - yaws[i]), math.cos(yaws[j] - yaws[i])
                ))
                if math.degrees(dyaw) > max_dyaw_deg:
                    continue
            distance = float(np.linalg.norm(xy[j] - xy[i]))
            candidates.append((distance, j - i, i, j))
    candidates.sort(key=lambda item: (item[0], -item[1]))
    pairs, used = [], np.zeros(n, dtype=bool)
    for _, _, i, j in candidates:
        if used[max(0, i - 30):i + 30].any() or used[max(0, j - 30):j + 30].any():
            continue
        pairs.append((i, j))
        used[i] = used[j] = True
        if len(pairs) >= 200:
            break
    return pairs


def evaluate(bag, map_dir, args):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_map import read_bag, voxel_downsample

    scans, scan_stamps, _, _ = read_bag(bag)
    stamps, base_poses = read_tum_poses(map_dir / "trajectory.tum")
    if len(stamps) != len(scans):
        raise SystemExit(f"bag has {len(scans)} scans but trajectory has {len(stamps)} poses")
    lever = np.array([float(v) for v in args.base_lidar.split(",")])
    lidar_poses = [pose.copy() for pose in base_poses]
    for pose in lidar_poses:
        pose[:3, 3] += pose[:3, :3] @ lever  # base_link -> lidar
    xy = np.asarray([pose[:2, 3] for pose in lidar_poses])
    yaws = np.asarray([
        math.atan2(pose[1, 0], pose[0, 0]) for pose in lidar_poses
    ])

    pairs = find_revisit_pairs(
        stamps, xy, args.radius, args.min_dt, yaws, args.max_dyaw
    )
    rows = []
    half = args.submap_half

    def submap_around(k):
        cloud = np.concatenate([
            scans[m] @ lidar_poses[m][:3, :3].T + lidar_poses[m][:3, 3]
            for m in range(max(0, k - half), min(len(scans), k + half + 1))
        ])
        return voxel_downsample(cloud, args.voxel)

    def scan_at(k):
        cloud = voxel_downsample(scans[k], args.voxel)
        return cloud @ lidar_poses[k][:3, :3].T + lidar_poses[k][:3, 3]  # map frame

    for i, j in pairs:
        submap_j = submap_around(j)
        source = scan_at(i)
        # Gated metric: the search-free inter-visit residual. Each visit's
        # scan, held at its RECORDED pose, is scored against the other
        # visit's submap by nearest-neighbour distance. Walked ICP was tried
        # first and is kept only as a diagnostic: on corridor geometry its
        # basin is so shallow that a 3 cm pose difference flips a 0.25 m lock
        # into a multi-metre slide, reporting basin fragility as map error.
        nn_ij = float(np.median(
            cKDTree(submap_j).query(source, k=1, workers=-1)[0]))
        submap_i = submap_around(i)
        nn_ji = float(np.median(
            cKDTree(submap_i).query(scan_at(j), k=1, workers=-1)[0]))
        residual = max(nn_ij, nn_ji)

        transform, inliers = icp_planar(
            source, cKDTree(submap_j), np.eye(4), args.max_corr)
        # source already sits in the map frame at the recorded lidar pose; the
        # ICP transform is therefore the correction of that pose, in the map frame.
        delta = transform  # aligned_pose = delta @ recorded pose
        trans = float(np.linalg.norm(delta[:3, 3]))
        yaw = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
        rows.append({
            "scan_i": i, "scan_j": j, "dt_s": float(stamps[j] - stamps[i]),
            "residual_m": residual, "nn_ij_m": nn_ij, "nn_ji_m": nn_ji,
            "correction_m": trans, "correction_yaw_deg": float(abs(yaw)),
            "inlier_ratio": inliers,
        })
        print(f"LOOP_PAIR i={i} j={j} dt={stamps[j] - stamps[i]:.0f}s "
              f"residual_m={residual:.3f} (ij {nn_ij:.3f} ji {nn_ji:.3f}) "
              f"walk_m={trans:.3f}",
              flush=True)

    corrections = np.asarray([row["residual_m"] for row in rows]) if rows else np.array([])
    median = float(np.median(corrections)) if len(rows) else float("inf")
    p95 = float(np.percentile(corrections, 95)) if len(rows) else float("inf")
    passed = len(rows) >= GATES["min_pairs"] and median <= GATES["median_max_m"] and p95 <= GATES["p95_max_m"]
    summary = {
        "schema_version": 1,
        "map_id": json.loads((map_dir / "map.yaml").read_text()).get("map_id"),
        "pairs": len(rows),
        "median_residual_m": median,
        "p95_residual_m": p95,
        "max_residual_m": float(corrections.max()) if len(rows) else None,
        "gates": GATES,
        "passed": bool(passed),
        "note": "search-free inter-visit residual: median NN distance of each "
                "visit's scan, held at its recorded pose, to the other "
                "visit's submap (bidirectional max). Misposed or doubled "
                "geometry shows up directly; walked-ICP corrections "
                "(correction_m) are kept as diagnostics only",
        "pairs_detail": rows,
    }
    (map_dir / "validation").mkdir(exist_ok=True)
    (map_dir / "validation" / "loop_consistency.json").write_text(json.dumps(summary, indent=2))

    if rows:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10), dpi=120)
        ax.plot(xy[:, 0], xy[:, 1], color="0.7", linewidth=0.8)
        norm = plt.Normalize(0, max(0.4, corrections.max()))
        for row, (i, j) in zip(rows, pairs):
            ax.plot([xy[i, 0], xy[j, 0]], [xy[i, 1], xy[j, 1]],
                    color=plt.cm.plasma(norm(row["residual_m"])), linewidth=1)
            ax.plot(*xy[i], marker="o", color=plt.cm.plasma(norm(row["residual_m"])))
        ax.set_aspect("equal")
        ax.set_title(f"loop residuals: median={median:.3f} m p95={p95:.3f} m ({len(rows)} pairs)")
        fig.tight_layout()
        fig.savefig(map_dir / "validation" / "loop_check.png")
        plt.close()

    print(
        f"LOOP_SUMMARY pairs={len(rows)} median_m={median:.3f} p95_m={p95:.3f} "
        f"max_m={summary['max_residual_m']} gates={GATES} passed={passed}"
    )
    return passed


def self_test():
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        tum = Path(tmp) / "pose.tum"
        tum.write_text(
            "0 1 2 3 0 0 0.7071067811865476 0.7071067811865476\n",
            encoding="utf-8",
        )
        _, poses = read_tum_poses(tum)
        assert abs(poses[0][0, 1] + 1.0) < 1e-6

    rng = np.random.default_rng(5)
    # corner of two walls: a single straight wall would let ICP slide freely
    wall_a = np.column_stack([np.full(4000, 5.0), rng.uniform(-10, 10, 4000), rng.uniform(0, 3, 4000)])
    wall_b = np.column_stack([rng.uniform(-10, 10, 4000), np.full(4000, 8.0), rng.uniform(0, 3, 4000)])
    corner = np.concatenate([wall_a, wall_b])
    theta = math.radians(1.5)
    c, s = math.cos(theta), math.sin(theta)
    moved = corner @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]).T + [0.4, 0.1, 0.0]
    transform, inliers = icp_planar(moved, cKDTree(corner), np.eye(4), 2.0)
    delta_t = np.linalg.norm(transform[:3, 3])
    assert abs(delta_t - math.hypot(0.4, 0.1)) < 1e-3, f"icp translation {delta_t}"
    assert abs(math.degrees(math.atan2(transform[1, 0], transform[0, 0])) - 1.5) < 1e-2
    print("LOOP_SELF_TEST ok")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", nargs="?")
    parser.add_argument("map_dir", nargs="?")
    parser.add_argument("--radius", type=float, default=10.0)
    parser.add_argument("--min-dt", type=float, default=60.0)
    parser.add_argument("--max-dyaw", type=float, default=45.0)
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--max-corr", type=float, default=1.5)
    parser.add_argument("--submap-half", type=int, default=45)
    parser.add_argument("--base-lidar", default="1.394,0.0,2.700")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.bag or not args.map_dir:
        parser.error("need <bag> <map_dir> (or --self-test)")
    if not evaluate(args.bag, Path(args.map_dir), args):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
