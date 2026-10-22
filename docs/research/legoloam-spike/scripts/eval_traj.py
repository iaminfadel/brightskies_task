import csv
import json
import math
import sys
import numpy as np

gt_path, est_path, out_json, out_png = sys.argv[1:5]

def load_gt(path):
    rows = list(csv.DictReader(open(path)))
    return {
        "t": np.array([float(r["t_abs"]) for r in rows]),
        "x": np.array([float(r["x"]) for r in rows]),
        "y": np.array([float(r["y"]) for r in rows]),
        "speed": np.array([float(r["speed_kmh"]) for r in rows]),
    }

def load_est(path):
    rows = list(csv.DictReader(open(path)))
    return {
        "t": np.array([float(r["stamp"]) for r in rows]),
        "x": np.array([float(r["x"]) for r in rows]),
        "y": np.array([float(r["y"]) for r in rows]),
    }

def interp_xy(gt, ts):
    return np.interp(ts, gt["t"], gt["x"]), np.interp(ts, gt["t"], gt["y"])

def umeyama(src, dst, with_scale=False):
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = dc.T @ sc / len(src)
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    scale = 1.0
    if with_scale:
        var_s = (sc ** 2).sum() / len(src)
        scale = np.trace(np.diag(S)) / var_s if var_s > 0 else 1.0
    t = mu_d - scale * R @ mu_s
    return scale, R, t

def path_len(x, y):
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))

gt = load_gt(gt_path)
est = load_est(est_path)
# moving segment: speed > 2 km/h, with 1 s padding
moving = gt["speed"] > 2.0
if not moving.any():
    raise SystemExit("no moving GT")
idx = np.where(moving)[0]
t0, t1 = gt["t"][idx[0]], gt["t"][idx[-1]]
mask = (est["t"] >= t0) & (est["t"] <= t1)
if mask.sum() < 5:
    raise SystemExit("too few estimate samples in moving segment")
ts = est["t"][mask]
ex = est["x"][mask]
ey = est["y"][mask]
gx, gy = interp_xy(gt, ts)
src = np.column_stack([ex, ey])
dst = np.column_stack([gx, gy])
scale, R, t = umeyama(src, dst, with_scale=False)
aligned = (R @ src.T).T + t
err = np.linalg.norm(aligned - dst, axis=1)
gt_len = path_len(gx, gy)
est_len = path_len(ex, ey)
res = {
    "gt_file": gt_path,
    "est_file": est_path,
    "samples": int(mask.sum()),
    "t_start": float(t0),
    "t_end": float(t1),
    "gt_path_m": gt_len,
    "est_path_m": est_len,
    "path_length_error_pct": 100.0 * (est_len - gt_len) / gt_len if gt_len > 0 else None,
    "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
    "ate_mean_m": float(np.mean(err)),
    "ate_max_m": float(np.max(err)),
    "ate_final_m": float(err[-1]),
    "scale": float(scale),
    "gt_speed_mean_kmh": float(np.mean(gt["speed"][idx])),
    "gt_speed_max_kmh": float(np.max(gt["speed"][idx])),
}
with open(out_json, "w") as f:
    json.dump(res, f, indent=2)
print(json.dumps(res, indent=2))

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(gt["x"], gt["y"], "k-", lw=2, label="GT")
    ax.plot(ex, ey, "r-", lw=1.2, label="estimate (raw)")
    ax.plot(aligned[:, 0], aligned[:, 1], "b--", lw=1.2, label="estimate (aligned)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("LeGO-LOAM trajectory vs CARLA GT")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    print("wrote", out_png)
except Exception as e:
    print("plot skipped", e)
