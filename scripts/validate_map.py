#!/usr/bin/env python3
"""Validate a map bundle against its held-out scans (contract #15).

Held-out scans are stored in the map frame at their SLAM poses.  Each scan is
matched to ``map.pcd`` starting from identity with the production GICP
settings (0.25 m voxel, 1.0 m coarse / 0.5 m fine correspondence, 30
iterations, Gauss-Newton-equivalent small_gicp alignment, 2 threads) and the
RMSE / inlier / Hessian-condition gates of the localization matcher.

The gate is: at least 80 % of the scans accepted and median RMSE <= 0.20 m.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import small_gicp
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def read_pcd(path):
    with open(path, "rb") as handle:
        while True:
            line = handle.readline().decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            if key == "DATA":
                break
        data = handle.read()
    return np.frombuffer(data, dtype=np.float32).reshape(-1, 3).astype(np.float64)


def align(target, target_tree, source, args, init, correspondence):
    return small_gicp.align(
        target,
        source,
        target_tree,
        init,
        "GICP",
        float(correspondence),
        args.num_threads,
        args.max_iterations,
        args.rotation_eps,
        args.translation_eps,
    )


def evaluate(map_dir, args):
    map_points = read_pcd(map_dir / "map.pcd")
    target, target_tree = small_gicp.preprocess_points(
        map_points, args.voxel, args.num_neighbors, args.num_threads
    )
    map_kd = cKDTree(map_points)
    heldout = json.loads((map_dir / "heldout" / "poses.json").read_text())

    rows = []
    for item in heldout:
        scan = read_pcd(map_dir / item["file"])
        pose = np.asarray(item["T_map_lidar"], dtype=np.float64).reshape(4, 4)
        sensor = pose[:3, 3]
        scan = scan[np.linalg.norm(scan - sensor, axis=1) <= args.max_range]
        source, _ = small_gicp.preprocess_points(
            scan, args.voxel, args.num_neighbors, args.num_threads
        )
        coarse = align(
            target, target_tree, source, args, np.eye(4), args.coarse_correspondence
        )
        fine = align(
            target,
            target_tree,
            source,
            args,
            coarse.T_target_source,
            args.fine_correspondence,
        )
        motion = np.linalg.inv(coarse.T_target_source) @ fine.T_target_source
        fine_motion_m = float(np.linalg.norm(motion[:3, 3]))
        fine_rotation_rad = float(Rotation.from_matrix(motion[:3, :3]).magnitude())
        stalled = (
            fine_motion_m <= args.fine_motion_gate_m
            and fine_rotation_rad <= math.radians(args.fine_rotation_gate_deg)
        )

        points = np.asarray(source.points())[:, :3]
        transformed = points @ fine.T_target_source[:3, :3].T + fine.T_target_source[:3, 3]
        distances, _ = map_kd.query(transformed, k=1)
        inlier_mask = distances <= args.fine_correspondence
        inliers = int(np.count_nonzero(inlier_mask))
        inlier_ratio = inliers / len(points) if len(points) else 0.0
        rmse = (
            float(np.sqrt(np.mean(np.square(distances[inlier_mask]))))
            if inliers
            else float("inf")
        )
        eigenvalues = np.linalg.eigvalsh(fine.H)
        condition = (
            float(eigenvalues[-1] / eigenvalues[0])
            if eigenvalues[0] > 0.0
            else float("inf")
        )

        converged = bool(fine.converged) or stalled
        accepted = (
            converged
            and rmse <= args.rmse_gate
            and inlier_ratio >= args.inlier_gate
            and condition < args.condition_gate
        )
        row = {
            "index": item["index"],
            "converged": converged,
            "stalled": stalled,
            "iterations": int(fine.iterations),
            "fine_motion_m": fine_motion_m,
            "fine_rotation_deg": math.degrees(fine_rotation_rad),
            "rmse_m": rmse,
            "inlier_ratio": inlier_ratio,
            "condition": condition,
            "accepted": accepted,
        }
        rows.append(row)
        print(
            "VALIDATE "
            + " ".join(f"{key}={value}" for key, value in row.items())
            + f" scan={item['file']}",
            flush=True,
        )

    rmses = [row["rmse_m"] for row in rows]
    accepted = sum(row["accepted"] for row in rows)
    fraction = accepted / len(rows) if rows else 0.0
    median_rmse = float(np.median(rmses)) if rmses else float("inf")
    passed = fraction >= args.min_accepted_fraction and median_rmse <= args.rmse_gate
    summary = {
        "schema_version": 1,
        "map_id": None,
        "scans": len(rows),
        "accepted": accepted,
        "accepted_fraction": fraction,
        "median_rmse_m": median_rmse,
        "mean_rmse_m": float(np.mean(rmses)) if rmses else float("inf"),
        "p95_rmse_m": float(np.percentile(rmses, 95)) if rmses else float("inf"),
        "min_accepted_fraction": args.min_accepted_fraction,
        "rmse_gate_m": args.rmse_gate,
        "passed": passed,
        "scans_detail": rows,
    }
    print(
        f"MAP_VALIDATION scans={len(rows)} accepted={accepted} "
        f"fraction={fraction:.3f} median_rmse_m={median_rmse:.4f} "
        f"mean_rmse_m={summary['mean_rmse_m']:.4f} "
        f"p95_rmse_m={summary['p95_rmse_m']:.4f} "
        f"fraction_gate={args.min_accepted_fraction:.2f} "
        f"rmse_gate_m={args.rmse_gate:.2f} passed={passed}",
        flush=True,
    )
    if args.json:
        Path(args.json).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return passed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir")
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--num-neighbors", type=int, default=10)
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--max-iterations", type=int, default=30)
    parser.add_argument("--rotation-eps", type=float, default=1e-4)
    parser.add_argument("--translation-eps", type=float, default=1e-4)
    parser.add_argument("--coarse-correspondence", type=float, default=1.0)
    parser.add_argument("--fine-correspondence", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=50.0)
    parser.add_argument("--rmse-gate", type=float, default=0.20)
    parser.add_argument("--inlier-gate", type=float, default=0.60)
    parser.add_argument("--condition-gate", type=float, default=1.0e6)
    parser.add_argument("--fine-motion-gate-m", type=float, default=0.02)
    parser.add_argument("--fine-rotation-gate-deg", type=float, default=0.5)
    parser.add_argument("--min-accepted-fraction", type=float, default=0.80)
    parser.add_argument("--json", default="")
    args = parser.parse_args()
    if not evaluate(Path(args.map_dir), args):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
