#!/usr/bin/env python3
"""Loop-consistency check of a map bundle against its mapping bag.

For every revisited place (scan pairs far apart in time, close in recorded
position), registers one scan against a submap built from the other visit's
neighbourhood, starting AT the recorded pose. The registration correction is
the inter-visit misalignment frozen into the map -- exactly the doubled-wall
defect the held-out residual gate cannot see (it compares a scan against its
own visit's geometry).

Gates: >= min_pairs valid pairs, median correction <= 0.15 m, p95 <= 0.40 m.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

LIDAR_TOPIC = "/carla/hero/lidar"
GATES = {"min_pairs": 10, "median_max_m": 0.15, "p95_max_m": 0.40}


def read_tum_poses(path):
    stamps, poses = [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        stamps.append(float(parts[0]))
        poses.append(np.array([
            [1, 0, 0, float(parts[1])],
            [0, 1, 0, float(parts[2])],
            [0, 0, 1, float(parts[3])],
            [0, 0, 0, 1],
        ]))
    return np.asarray(stamps), poses


def icp(source, target_tree, init, max_corr, iterations=30):
    """Point-to-point ICP; returns (4x4, inlier_ratio). ponytail: plain ICP --
    KISS-ICP itself is point-to-point; adequate for ~1 m init errors."""
    transform = init.copy()
    prev_t = None
    for _ in range(iterations):
        moved = source @ transform[:3, :3].T + transform[:3, 3]
        distances, indices = target_tree.query(moved, k=1, workers=-1)
        mask = distances <= max_corr
        if mask.sum() < 20:
            break
        src_corr = source[mask]
        dst_corr = target_tree.data[indices[mask]]
        mu_s, mu_d = src_corr.mean(axis=0), dst_corr.mean(axis=0)
        u, _, vt = np.linalg.svd((dst_corr - mu_d).T @ (src_corr - mu_s))
        s = np.eye(3)
        if np.linalg.det(u) * np.linalg.det(vt) < 0:
            s[-1, -1] = -1.0
        rotation = u @ s @ vt
        step = np.eye(4)
        step[:3, :3] = rotation
        step[:3, 3] = mu_d - rotation @ mu_s
        transform = step  # step is the full source->target estimate, not a delta
        if prev_t is not None and np.linalg.norm(step[:3, 3] - prev_t) < 1e-4:
            break
        prev_t = step[:3, 3]
    moved = source @ transform[:3, :3].T + transform[:3, 3]
    distances, _ = target_tree.query(moved, k=1, workers=-1)
    return transform, float(np.mean(distances <= max_corr))


def find_revisit_pairs(stamps, xy, radius_m, min_dt_s):
    """Greedy revisit pairs: close in space, far apart in time."""
    tree = cKDTree(xy)
    candidates = []
    n = len(xy)
    for i in range(n):
        for j in tree.query_ball_point(xy[i], radius_m):
            if j - i > 50 and stamps[j] - stamps[i] >= min_dt_s:
                candidates.append((j - i, i, j))
    candidates.sort(reverse=True)
    pairs, used = [], np.zeros(n, dtype=bool)
    for gap, i, j in candidates:
        if used[max(0, i - 30):i + 30].any() or used[max(0, j - 30):j + 30].any():
            continue
        pairs.append((i, j))
        used[i], used[j] = True
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
    lever = np.array(args.base_lidar, dtype=float)
    lidar_poses = [pose.copy() for pose in base_poses]
    for pose in lidar_poses:
        pose[:3, 3] += pose[:3, :3] @ lever  # base_link -> lidar
    xy = np.asarray([pose[:2, 3] for pose in lidar_poses])

    pairs = find_revisit_pairs(stamps, xy, args.radius, args.min_dt)
    rows = []
    half = args.submap_half
    for i, j in pairs:
        submap = np.concatenate([
            scans[k] @ lidar_poses[k][:3, :3].T + lidar_poses[k][:3, 3]
            for k in range(max(0, j - half), min(len(scans), j + half + 1))
        ])
        submap = voxel_downsample(submap, args.voxel)
        source = voxel_downsample(scans[i], args.voxel)
        source = source @ lidar_poses[i][:3, :3].T + lidar_poses[i][:3, 3]  # map frame
        transform, inliers = icp(source, cKDTree(submap), np.eye(4), args.max_corr)
        # source already sits in the map frame at the recorded lidar pose; the
        # ICP transform is therefore the correction of that pose, in the map frame.
        delta = transform  # aligned_pose = delta @ recorded pose
        trans = float(np.linalg.norm(delta[:3, 3]))
        yaw = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
        rows.append({
            "scan_i": i, "scan_j": j, "dt_s": float(stamps[j] - stamps[i]),
            "correction_m": trans, "correction_yaw_deg": float(abs(yaw)),
            "inlier_ratio": inliers,
        })
        print(f"LOOP_PAIR i={i} j={j} dt={stamps[j] - stamps[i]:.0f}s "
              f"correction_m={trans:.3f} yaw_deg={abs(yaw):.2f} inliers={inliers:.2f}",
              flush=True)

    corrections = np.asarray([row["correction_m"] for row in rows]) if rows else np.array([])
    median = float(np.median(corrections)) if len(rows) else float("inf")
    p95 = float(np.percentile(corrections, 95)) if len(rows) else float("inf")
    passed = len(rows) >= GATES["min_pairs"] and median <= GATES["median_max_m"] and p95 <= GATES["p95_max_m"]
    summary = {
        "schema_version": 1,
        "map_id": json.loads((map_dir / "map.yaml").read_text()).get("map_id"),
        "pairs": len(rows),
        "median_correction_m": median,
        "p95_correction_m": p95,
        "max_correction_m": float(corrections.max()) if len(rows) else None,
        "gates": GATES,
        "passed": bool(passed),
        "note": "ICP correction of the recorded pose against the other visit's submap; "
                "doubled walls in the map show up as large corrections",
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
                    color=plt.cm.plasma(norm(row["correction_m"])), linewidth=1)
            ax.plot(*xy[i], marker="o", color=plt.cm.plasma(norm(row["correction_m"])))
        ax.set_aspect("equal")
        ax.set_title(f"loop corrections: median={median:.3f} m p95={p95:.3f} m ({len(rows)} pairs)")
        fig.tight_layout()
        fig.savefig(map_dir / "validation" / "loop_check.png")
        plt.close(fig)

    print(
        f"LOOP_SUMMARY pairs={len(rows)} median_m={median:.3f} p95_m={p95:.3f} "
        f"max_m={summary['max_correction_m']} gates={GATES} passed={passed}"
    )
    return passed


def self_test():
    rng = np.random.default_rng(5)
    # corner of two walls: a single straight wall would let ICP slide freely
    wall_a = np.column_stack([np.full(4000, 5.0), rng.uniform(-10, 10, 4000), rng.uniform(0, 3, 4000)])
    wall_b = np.column_stack([rng.uniform(-10, 10, 4000), np.full(4000, 8.0), rng.uniform(0, 3, 4000)])
    corner = np.concatenate([wall_a, wall_b])
    theta = math.radians(1.5)
    c, s = math.cos(theta), math.sin(theta)
    moved = corner @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]).T + [0.4, 0.1, 0.0]
    transform, inliers = icp(moved, cKDTree(corner), np.eye(4), 2.0)
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
