#!/usr/bin/env python3
"""Minimal SE(2)+z pose-graph optimizer for the offline mapper.

Nodes are keyframe poses (x, y, yaw, z) of the LiDAR. Edges:
  odometry  -- relative pose between consecutive KISS-ICP keyframes
  gnss      -- unary position constraints from the GNSS (lever-arm corrected)
  loop      -- relative poses measured by ICP of a revisited scan against the
               other visit's submap

Solver: sparse Gauss-Newton, Huber weights on gnss/loop edges. Roads are
close to planar, so SE(2)+z covers this domain; full SE(3) is the upgrade
path if elevation-heavy towns show residual z error.
"""
import math

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

ODOM_INFO = np.diag([40.0, 40.0, 60.0, 20.0])   # 1/sigma^2
GNSS_INFO = np.diag([25.0, 25.0, 4.0])          # (x, y, z)
LOOP_INFO = np.diag([100.0, 100.0, 150.0, 50.0])
HUBER_DELTA = 3.0  # in residual sigmas


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def state_to_matrix(state):
    x, y, yaw, z = state
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0, x], [s, c, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])


def matrix_to_state(pose):
    return np.array([pose[0, 3], pose[1, 3], math.atan2(pose[1, 0], pose[0, 0]), pose[2, 3]])


def relative(a, b):
    """Pose of b expressed in a's frame: (dx, dy, dyaw, dz)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx_w, dy_w = b[0] - a[0], b[1] - a[1]
    return np.array([c * dx_w + s * dy_w, -s * dx_w + c * dy_w,
                     wrap(b[2] - a[2]), b[3] - a[3]])


def relative_jacobians(a, b):
    """Jacobians of relative(a, b) wrt a and b (4x4 each, state order x,y,yaw,z)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    jac_a = np.array([
        [-c, -s, s * dx - c * dy, 0.0],
        [s, -c, -(c * dx + s * dy), 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, -1.0],
    ])
    jac_b = np.array([
        [c, s, 0.0, 0.0],
        [-s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return jac_a, jac_b


def huber_weight(residual, info_diag, delta=HUBER_DELTA):
    scale = 1.0 / np.sqrt(np.maximum(info_diag, 1e-9))
    norm = np.linalg.norm(residual * scale / delta)
    return 1.0 if norm <= 1.0 else 1.0 / norm


def optimize(states, odom_edges, gnss_edges, loop_edges, iterations=30, verbose=True):
    """states: (n,4) initial guess.
    odom/loop edges: {"i", "j", "m": (dx,dy,dyaw,dz)}; gnss: {"i", "p": (x,y,z)}.
    Returns optimized (n,4)."""
    states = np.asarray(states, dtype=float).copy()
    n = len(states)

    def accumulate(h, b, blocks, residual, info, weight):
        weighted = info * weight
        for a_index, jac_a in blocks:
            for c_index, jac_c in blocks:
                h[4 * a_index:4 * a_index + 4, 4 * c_index:4 * c_index + 4] += (
                    jac_a.T @ weighted @ jac_c)
            b[4 * a_index:4 * a_index + 4] += jac_a.T @ weighted @ residual

    for iteration in range(iterations):
        h = sp.lil_matrix((4 * n, 4 * n))
        b = np.zeros(4 * n)

        for edge in odom_edges:
            i, j, m = edge["i"], edge["j"], np.asarray(edge["m"], dtype=float)
            residual = relative(states[i], states[j]) - m
            residual[2] = wrap(residual[2])
            jac_a, jac_b = relative_jacobians(states[i], states[j])
            accumulate(h, b, [(i, jac_a), (j, jac_b)], residual, ODOM_INFO, 1.0)

        for edge in gnss_edges:
            i, p = edge["i"], np.asarray(edge["p"], dtype=float)
            residual = np.array([states[i, 0] - p[0], states[i, 1] - p[1], states[i, 3] - p[2]])
            weight = huber_weight(residual, np.diag(GNSS_INFO))
            jac = np.zeros((3, 4))  # residual = selector @ state
            jac[0, 0] = jac[1, 1] = jac[2, 3] = 1.0
            accumulate(h, b, [(i, jac)], residual, GNSS_INFO, weight)

        for edge in loop_edges:
            i, j, m = edge["i"], edge["j"], np.asarray(edge["m"], dtype=float)
            residual = relative(states[i], states[j]) - m
            residual[2] = wrap(residual[2])
            jac_a, jac_b = relative_jacobians(states[i], states[j])
            weight = huber_weight(residual, np.diag(LOOP_INFO))
            accumulate(h, b, [(i, jac_a), (j, jac_b)], residual, LOOP_INFO, weight)

        # Anchor the gauge: pin the full first state (x, y, yaw, z).
        for d in range(4):
            h[d, d] += 1.0

        delta = spla.spsolve(h.tocsc(), -b)
        if not np.all(np.isfinite(delta)):
            break
        update = delta.reshape(-1, 4)
        # Trust region: a healthy GN step is sub-metre; huge steps mean a bad
        # edge or ill-conditioning, so clamp rather than diverge.
        scale_xy = np.abs(update[:, :2]).max()
        if scale_xy > 20.0:
            update *= 20.0 / scale_xy
            if verbose:
                print(f"  pose_graph iter {iteration}: clamped step {scale_xy:.1f} m -> 20 m", flush=True)
        states[:, 0] += update[:, 0]
        states[:, 1] += update[:, 1]
        states[:, 2] = wrap(states[:, 2] + update[:, 2])
        states[:, 3] += update[:, 3]
        if verbose:
            print(f"  pose_graph iter {iteration}: max_xy_update "
                  f"{np.abs(update[:, :2]).max():.4f} m", flush=True)
        if np.abs(update[:, :2]).max() < 1e-4:
            break
    return states


def build_loop_edges(scans, states, stamps, submap_half=45, radius=10.0,
                     min_dt=60.0, voxel=0.25, max_corr=1.5, max_pairs=200):
    """ICP loop-closure edges between revisited keyframes (uses current states
    for gating; measurements are scan-to-submap registrations)."""
    from pathlib import Path
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from build_map import voxel_downsample
    from eval_loops import icp
    from scipy.spatial import cKDTree

    xy = states[:, :2]
    tree = cKDTree(xy)
    candidates = []
    for i in range(len(xy)):
        for j in tree.query_ball_point(xy[i], radius):
            if j - i > 5 and stamps[j] - stamps[i] >= min_dt:
                candidates.append((j - i, i, j))
    candidates.sort(reverse=True)
    pairs, used = [], np.zeros(len(xy), dtype=bool)
    for _, i, j in candidates:
        if used[max(0, i - 3):i + 3].any() or used[max(0, j - 3):j + 3].any():
            continue
        pairs.append((i, j))
        used[i] = used[j] = True
        if len(pairs) >= max_pairs:
            break

    def scan_in_world(k, scan):
        pose = state_to_matrix(states[k])
        return scan @ pose[:3, :3].T + pose[:3, 3]

    edges = []
    for i, j in pairs:
        submap = np.concatenate([
            scan_in_world(k, scans[k])
            for k in range(max(0, j - submap_half), min(len(scans), j + submap_half + 1))
        ])
        submap = voxel_downsample(submap, voxel)
        source = voxel_downsample(scans[i], voxel)
        source_map = scan_in_world(i, source)
        transform, inliers = icp(source_map, cKDTree(submap), np.eye(4), max_corr)
        if inliers < 0.6 or np.linalg.norm(transform[:3, 3]) > 8.0:
            print(f"LOOP_EDGE_REJECTED i={i} j={j} correction="
                  f"{np.linalg.norm(transform[:3, 3]):.3f}m inliers={inliers:.2f}", flush=True)
            continue
        aligned_i = matrix_to_state(transform @ state_to_matrix(states[i]))
        edges.append({
            "i": i, "j": j,
            "m": relative(aligned_i, states[j]),
            "correction_m": float(np.linalg.norm(transform[:3, 3])),
            "inliers": inliers,
        })
        print(f"LOOP_EDGE i={i} j={j} correction={edges[-1]['correction_m']:.3f}m "
              f"inliers={inliers:.2f}", flush=True)
    return edges


def self_test():
    rng = np.random.default_rng(7)
    n = 300
    truth = []
    for k in range(n):
        side, t = divmod(k, n // 4)
        t = t / (n // 4) * 100.0
        truth.append([t, 0.0, 0.0, 0.0] if side == 0 else
                     [100.0, t, math.pi / 2, 0.0] if side == 1 else
                     [100.0 - t, 100.0, math.pi, 0.0] if side == 2 else
                     [0.0, 100.0 - t, -math.pi / 2, 0.0])
    truth = np.asarray(truth)

    # Drifted odometry: 0.3 deg/step yaw bias + 2 cm noise per edge.
    odom_states = np.zeros_like(truth)
    odom_edges = []
    for k in range(1, n):
        m = relative(truth[k - 1], truth[k])
        m[2] += math.radians(0.3)
        m[:2] += rng.normal(0, 0.02, 2)
        odom_edges.append({"i": k - 1, "j": k, "m": m})
        c, s = math.cos(odom_states[k - 1, 2]), math.sin(odom_states[k - 1, 2])
        odom_states[k, 0] = odom_states[k - 1, 0] + c * m[0] - s * m[1]
        odom_states[k, 1] = odom_states[k - 1, 1] + s * m[0] + c * m[1]
        odom_states[k, 2] = wrap(odom_states[k - 1, 2] + m[2])
        odom_states[k, 3] = 0.0

    gnss_edges = [{"i": k, "p": [truth[k, 0] + rng.normal(0, 0.2),
                                 truth[k, 1] + rng.normal(0, 0.2), 0.0]}
                  for k in range(0, n, 5)]
    loop_edges = [{"i": n - 5, "j": 5, "m": relative(truth[n - 5], truth[5])}]

    raw = np.linalg.norm(odom_states[:, :2] - truth[:, :2], axis=1)
    optimized = optimize(odom_states, odom_edges, gnss_edges, loop_edges, verbose=False)
    opt = np.linalg.norm(optimized[:, :2] - truth[:, :2], axis=1)
    print(f"pose_graph self-test: raw rmse {np.sqrt((raw ** 2).mean()):.2f} m "
          f"-> optimized rmse {np.sqrt((opt ** 2).mean()):.2f} m")
    assert np.sqrt((opt ** 2).mean()) < 0.5, "optimizer failed to correct drift"
    print("POSE_GRAPH_SELF_TEST ok")


if __name__ == "__main__":
    self_test()
