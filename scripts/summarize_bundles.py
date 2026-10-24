#!/usr/bin/env python3
"""Summarize the mapping-methods comparison across map bundles.

Reads each bundle's map.yaml / trajectory.tum / validation/{ate,loop_consistency}
JSON, computes z-drift and closure diagnostics, and writes comparison.json and
comparison.md next to the bundles.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_tum(path):
    poses = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) != 8:
            continue
        poses.append(
            (
                float(parts[0]),
                float(parts[1]),
                float(parts[2]),
                float(parts[3]),
            )
        )
    return np.asarray(poses)


def bundle_summary(out_dir, name):
    bundle = out_dir / name
    if not (bundle / "map.pcd").is_file():
        return None
    summary = {"name": name}
    map_yaml = json.loads((bundle / "map.yaml").read_text())
    summary["point_count"] = map_yaml.get("point_count")
    summary["pose_source"] = map_yaml.get("source", {}).get("pose_source")
    summary["trajectory_length_m"] = map_yaml.get("source", {}).get(
        "trajectory_length_m"
    )

    tum = read_tum(bundle / "trajectory.tum")
    xy = tum[:, 1:3]
    z = tum[:, 3]
    steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    closure = float(np.linalg.norm(tum[-1, 1:3] - tum[0, 1:3]))
    summary["z_min_m"] = float(z.min())
    summary["z_max_m"] = float(z.max())
    summary["z_range_m"] = float(np.ptp(z))
    summary["closure_m"] = closure
    summary["path_xy_m"] = float(np.sum(steps))
    if map_yaml.get("source", {}).get("trajectory_length_m"):
        path3d = map_yaml["source"]["trajectory_length_m"]
        summary["z_share_pct"] = (
            100.0 * math.sqrt(max(path3d ** 2 - summary["path_xy_m"] ** 2, 0.0)) / path3d
        )

    ate_path = bundle / "validation" / "ate.json"
    if ate_path.is_file():
        ate = json.loads(ate_path.read_text())
        for key in ("abs_rmse_m", "abs_p95_m", "abs_max_m", "aligned_rmse_m",
                    "aligned_max_m", "yaw_rmse_deg", "poses", "passed"):
            summary[f"ate_{key}"] = ate.get(key)

    loops_path = bundle / "validation" / "loop_consistency.json"
    if loops_path.is_file():
        loops = json.loads(loops_path.read_text())
        for key in ("pairs", "median_correction_m", "p95_correction_m",
                    "max_correction_m", "passed"):
            summary[f"loops_{key}"] = loops.get(key)
    return summary


COLUMNS = [
    ("name", "bundle"),
    ("ate_abs_rmse_m", "ATE rmse m"),
    ("ate_abs_p95_m", "ATE p95 m"),
    ("ate_aligned_rmse_m", "drift rmse m"),
    ("ate_yaw_rmse_deg", "yaw rmse deg"),
    ("loops_median_correction_m", "loop med m"),
    ("loops_p95_correction_m", "loop p95 m"),
    ("z_range_m", "z range m"),
    ("closure_m", "closure m"),
    ("path_xy_m", "xy path m"),
    ("point_count", "map points"),
]


def fmt(value):
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def main():
    out_dir = Path(sys.argv[1])
    names = sys.argv[2:] or [
        "baseline-kiss",
        "baseline-kiss-pg",
        "gicp-pg",
        "lego",
        "lego-zhold",
    ]
    rows = []
    for name in names:
        row = bundle_summary(out_dir, name)
        if row:
            rows.append(row)
        else:
            print(f"summary: {name} missing, skipped")

    (out_dir / "comparison.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8"
    )
    header = "| " + " | ".join(label for _, label in COLUMNS) + " |"
    divider = "|" + "|".join("---" for _ in COLUMNS) + "|"
    lines = [header, divider]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(key)) for key, _ in COLUMNS) + " |")
    report = "\n".join(lines) + "\n"
    (out_dir / "comparison.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
