#!/usr/bin/env python3
"""Render visual evidence for a saved-map bundle."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch

plt.rcParams.update(
    {
        "figure.facecolor": "#0b0e14",
        "axes.facecolor": "#0b0e14",
        "axes.edgecolor": "#3a4150",
        "axes.labelcolor": "#d7dce5",
        "text.color": "#d7dce5",
        "xtick.color": "#8b94a5",
        "ytick.color": "#8b94a5",
        "font.size": 10,
    }
)


def read_pcd(path):
    data = Path(path).read_bytes()
    header_end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    header = data[:header_end].decode("ascii")
    fields = None
    points = 0
    for line in header.splitlines():
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("POINTS"):
            points = int(line.split()[1])
    count = len(fields)
    cloud = np.frombuffer(data[header_end:], dtype=np.float32,
                          count=points * count).reshape(points, count)
    return cloud


def read_tum(path):
    poses = np.loadtxt(path)
    return poses[:, 1:4]


def main():
    if len(sys.argv) != 3:
        print("usage: render_map_evidence.py <bundle_dir> <out_dir>", file=sys.stderr)
        return 2
    bundle = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)

    cloud = read_pcd(bundle / "map.pcd")
    trajectory = read_tum(bundle / "trajectory.tum")
    map_id = json.loads((bundle / "map.yaml").read_text()).get("map_id", "map")
    consistency = json.loads((bundle / "validation" / "map_consistency.json").read_text())
    start, end = trajectory[0], trajectory[-1]
    closure = float(np.linalg.norm(end[:2] - start[:2]))

    def maybe_json(name):
        try:
            return json.loads((bundle / "validation" / name).read_text())
        except OSError:
            return None

    ate = maybe_json("ate.json")
    loops = maybe_json("loop_consistency.json")
    extra_lines = ""
    if ate:
        extra_lines += (
            f"\nATE vs GT  rmse {ate['abs_rmse_m']:.2f} m  p95 {ate['abs_p95_m']:.2f} m"
            f"  yaw {ate['yaw_rmse_deg']:.1f} deg  [{'PASS' if ate['passed'] else 'FAIL'}]"
        )
    if loops:
        extra_lines += (
            f"\nloop consistency  {loops['pairs']} pairs  median {loops['median_residual_m']:.2f} m"
            f"  p95 {loops['p95_residual_m']:.2f} m  [{'PASS' if loops['passed'] else 'FAIL'}]"
        )

    # 1. Overview ---------------------------------------------------------
    fig, ax = plt.subplots(figsize=(15, 12), dpi=150)
    stride = max(1, len(cloud) // 900000)
    shown = cloud[::stride]
    ax.scatter(shown[:, 0], shown[:, 1], s=0.08, c=shown[:, 2], cmap="viridis",
               vmin=-3, vmax=3, linewidths=0, rasterized=True, alpha=0.9)
    ax.plot(trajectory[:, 0], trajectory[:, 1], color="#ffb000", linewidth=2.0,
            label="trajectory")
    ax.plot(*start[:2], marker="o", color="#39ff88", markersize=9, label="start")
    ax.plot(*end[:2], marker="X", color="#ff3860", markersize=11, label="end")
    ax.annotate("", xy=end[:2], xytext=start[:2],
                arrowprops=dict(arrowstyle="->", color="#ff3860", lw=1.6))
    ax.text(0.02, 0.98,
            f"{map_id}  {len(cloud):,} points  {len(trajectory)} poses\n"
            f"closure {closure:.2f} m  held-out {consistency['accepted']}/"
            f"{consistency['scans']} accepted  median RMSE "
            f"{consistency['median_rmse_m']:.3f} m" + extra_lines,
            transform=ax.transAxes, va="top", fontsize=11, color="#e8edf5",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#141a23",
                      edgecolor="#3a4150"))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Saved map and mapping trajectory", fontsize=14)
    ax.grid(color="#1d2430", linewidth=0.6)
    ax.legend(loc="lower right", framealpha=0.2)
    fig.tight_layout()
    fig.savefig(out / "map_overview.png")
    plt.close(fig)

    # 2. Perspective ------------------------------------------------------
    fig = plt.figure(figsize=(15, 11), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#0b0e14")
    sample = cloud[::max(1, len(cloud) // 250000)]
    ax.scatter(sample[:, 0], sample[:, 1], sample[:, 2], s=0.15,
               c=sample[:, 2], cmap="viridis", vmin=-3, vmax=3, linewidths=0)
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color="#ffb000", linewidth=2.0)
    ax.scatter(*start, color="#39ff88", s=60, depthshade=False)
    ax.scatter(*end, color="#ff3860", s=60, marker="X", depthshade=False)
    ax.view_init(elev=32, azim=-62)
    ax.set_box_aspect((np.ptp(cloud[:, 0]), np.ptp(cloud[:, 1]), 140))
    ax.set_xlabel("x [m]", labelpad=8)
    ax.set_ylabel("y [m]", labelpad=8)
    ax.set_zlabel("z [m]", labelpad=8)
    ax.set_title(f"{map_id} perspective view (height-colored)", fontsize=14)
    fig.tight_layout()
    fig.savefig(out / "map_perspective.png")
    plt.close(fig)

    # 3. Held-out alignment ----------------------------------------------
    scans = consistency["scans_detail"]
    indices = [item["index"] for item in scans]
    rmse = [item["rmse_m"] for item in scans]
    innovation = [item["innovation_d2"] for item in scans]
    accepted = [item["accepted"] for item in scans]
    colors = ["#39ff88" if ok else "#ff3860" for ok in accepted]
    fig, ax = plt.subplots(figsize=(13, 5.5), dpi=150)
    ax.bar(range(len(scans)), rmse, color=colors, alpha=0.85)
    ax.axhline(consistency["gates"]["median_rmse_max_m"], color="#ffb000",
               linestyle="--", linewidth=1.4, label="RMSE gate 0.20 m")
    ax.set_xticks(range(len(scans)))
    ax.set_xticklabels([str(index) for index in indices], rotation=45, fontsize=8)
    ax.set_xlabel("held-out scan index")
    ax.set_ylabel("GICP RMSE [m]")
    ax.set_title(
        f"Held-out scan alignment: {consistency['accepted']}/{consistency['scans']} "
        f"accepted, median {consistency['median_rmse_m']:.3f} m"
    )
    ax.grid(axis="y", color="#1d2430")
    ax.legend(loc="upper left")
    ax2 = ax.twinx()
    ax2.plot(range(len(scans)), innovation, color="#5aa9ff", marker="o",
             markersize=3, linewidth=1.0, label="innovation d$^2$")
    ax2.axhline(consistency["gates"]["innovation_d2"], color="#5aa9ff",
                linestyle=":", linewidth=1.2, label="d$^2$ gate")
    ax2.set_ylabel("innovation d$^2$", color="#5aa9ff")
    ax2.tick_params(axis="y", colors="#5aa9ff")
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, loc="upper left", framealpha=0.2)
    fig.tight_layout()
    fig.savefig(out / "heldout_rmse.png")
    plt.close(fig)

    # 4. Closure zoom -----------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 8), dpi=150)
    centre = (start[:2] + end[:2]) / 2.0
    radius = max(15.0, closure * 4.0)
    mask = (np.abs(cloud[:, 0] - centre[0]) < radius) & (
        np.abs(cloud[:, 1] - centre[1]) < radius
    )
    local = cloud[mask]
    ax.scatter(local[:, 0], local[:, 1], s=0.6, c=local[:, 2], cmap="viridis",
               vmin=-3, vmax=3, linewidths=0)
    ax.plot(trajectory[:, 0], trajectory[:, 1], color="#ffb000", linewidth=2.0)
    ax.plot(*start[:2], marker="o", color="#39ff88", markersize=10)
    ax.plot(*end[:2], marker="X", color="#ff3860", markersize=11)
    arrow = FancyArrowPatch(start[:2], end[:2], arrowstyle="<->",
                            color="#ff3860", mutation_scale=14, linewidth=1.6)
    ax.add_patch(arrow)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Loop closure: {closure:.2f} m")
    ax.grid(color="#1d2430", linewidth=0.6)
    fig.tight_layout()
    fig.savefig(out / "closure_zoom.png")
    plt.close(fig)

    # 5. Held-out scan against the map -----------------------------------
    scan_files = {int(path.stem.split("_")[1]): path
                  for path in (bundle / "validation" / "heldout" / "scans").glob("*.pcd")}
    wanted = [min(scan_files), sorted(scan_files)[len(scan_files) // 2],
              max(scan_files)]
    picks = [scan_files[index] for index in wanted if index in scan_files]
    fig, axes = plt.subplots(1, len(picks), figsize=(15, 6.2), dpi=150)
    for ax, scan_path in zip(np.atleast_1d(axes), picks):
        scan = read_pcd(scan_path)
        centre = scan[:, :2].mean(axis=0)
        half = 18.0
        mask = (np.abs(cloud[:, 0] - centre[0]) < half) & (
            np.abs(cloud[:, 1] - centre[1]) < half
        )
        local = cloud[mask][:, :2] - centre
        ax.scatter(local[:, 0], local[:, 1], s=0.6, c="#7d879b", linewidths=0,
                   alpha=0.7, label="map")
        ax.scatter(scan[:, 0] - centre[0], scan[:, 1] - centre[1], s=0.9,
                   c="#39ff88", linewidths=0, alpha=0.75, label="held-out scan")
        index = int(scan_path.stem.split("_")[1])
        item = next((s for s in scans if s["index"] == index), None)
        title = f"scan {index:04d}"
        if item:
            title += (f"  RMSE {item['rmse_m']:.3f} m  "
                      f"inliers {item['inlier_ratio']:.2f}")
        ax.set_title(title, fontsize=11)
        ax.set_aspect("equal")
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_xlabel("x - scan centre [m]")
        ax.set_ylabel("y - scan centre [m]")
        ax.grid(color="#1d2430", linewidth=0.5)
    axes = np.atleast_1d(axes)
    axes[0].legend(loc="upper right", framealpha=0.2, markerscale=8)
    fig.suptitle("Held-out scans (green) registered against the saved map (grey)",
                 fontsize=14)
    fig.tight_layout()
    fig.savefig(out / "scan_vs_map.png")
    plt.close(fig)

    print(f"rendered evidence to {out}")
    import shutil

    for name in ("ate.png", "loop_check.png"):
        source = bundle / "validation" / name
        if source.is_file():
            shutil.copy(source, out / name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
