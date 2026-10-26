#!/usr/bin/env python3
"""Offline heading, output-age, and latency-drift KPIs for a localization run.

Evaluation path only.  Reads a completed estimator bag and the separate
ground-truth bag, joins estimator pose and GT by simulation time, and writes
``metrics.json`` / ``metrics.csv`` / ``metrics_summary.md`` plus a plot for
the evidence bundle.  ``kpi_gate.py`` owns the thresholds and is reused here
so a run's verdict cannot drift from the CI gate.

Contract (#11):

* Heading error is the wrapped yaw difference between ``/odometry/filtered``
  (``map -> base_link``) and the GT hero pose transformed into ``map`` with
  ``T_map_carla``.  Both poses are lifted to the CARLA actor origin with the
  static ``base_link -> hero`` lever so the comparison is like-for-like.
* Output age is the simulation-time bag reception time of each
  ``/odometry/filtered`` message minus its header stamp.  Bag timestamps are
  wall time during a recording, so ``/clock`` converts them back to sim time;
  ``/localization/status.latency_ms`` is reported as a cross-check.
* Latency drift is the p95 of the last 30 s minus the p95 of the first 30 s
  (last 20% minus first 20% when the metric window is shorter than 60 s).
"""
import argparse
import csv
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kpi_gate  # noqa: E402  (same directory, deliberately imported by path)

POSE_TOPIC = "/odometry/filtered"
GT_TOPIC = "/carla/hero/odometry"
STATUS_TOPIC = "/localization/status"
CLOCK_TOPIC = "/clock"
GNSS_TOPIC = "/carla/hero/gnss"
TF_STATIC_TOPIC = "/tf_static"

ROOT = Path(__file__).resolve().parent.parent
STATUS_MSG = ROOT / "src/brightdrive_interfaces/msg/LocalizationStatus.msg"

METRIC_KEYS = (
    "heading_rmse_deg",
    "heading_p95_deg",
    "heading_max_deg",
    "latency_p95_ms",
    "latency_max_ms",
    "latency_drift_ms",
)


# --------------------------------------------------------------- basic math


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def p95(values):
    return float(np.percentile(values, 95)) if len(values) else None


def nearest_pairs(reference, query):
    """For each reference stamp, index and dt of the nearest query stamp."""
    order = np.argsort(query)
    query_sorted = query[order]
    positions = np.searchsorted(query_sorted, reference)
    left = np.clip(positions - 1, 0, len(query_sorted) - 1)
    right = np.clip(positions, 0, len(query_sorted) - 1)
    choose_left = np.abs(reference - query_sorted[left]) <= np.abs(
        reference - query_sorted[right]
    )
    nearest = np.where(choose_left, left, right)
    return order[nearest], np.abs(reference - query_sorted[nearest])


def clock_to_sim(bag_stamps_ns, clock_bag_ns, clock_sim_s):
    """Map wall bag timestamps to simulation time by linear interpolation."""
    if len(clock_bag_ns) < 2:
        return np.full(len(bag_stamps_ns), np.nan)
    order = np.argsort(clock_bag_ns)
    clock_bag_ns = np.asarray(clock_bag_ns, dtype=np.float64)[order]
    clock_sim_s = np.asarray(clock_sim_s, dtype=np.float64)[order]
    stamps = np.asarray(bag_stamps_ns, dtype=np.float64)
    inside = (stamps >= clock_bag_ns[0]) & (stamps <= clock_bag_ns[-1])
    sim = np.full(len(stamps), np.nan)
    sim[inside] = np.interp(stamps[inside], clock_bag_ns, clock_sim_s)
    return sim


# ------------------------------------------------------------ scoring logic


def score(heading_t, heading_err_deg, latency_t, latency_ms,
          scenario, settle_s=2.0, min_samples=None):
    """Window the samples and compute the run metrics + validity reasons."""
    reasons = []
    heading_t = np.asarray(heading_t, dtype=np.float64)
    heading_err_deg = np.asarray(heading_err_deg, dtype=np.float64)
    latency_t = np.asarray(latency_t, dtype=np.float64)
    latency_ms = np.asarray(latency_ms, dtype=np.float64)

    if len(heading_t) < 2:
        return {}, ["no paired heading samples"]
    order = np.argsort(heading_t)
    heading_t, heading_err_deg = heading_t[order], heading_err_deg[order]

    start = float(heading_t[0] + settle_s)
    end = float(heading_t[-1])
    if end - start <= 0:
        return {}, [f"metric window too short ({end - start:.3f} s after settle)"]

    window = (heading_t >= start) & (heading_t <= end)
    heading_t, heading_err_deg = heading_t[window], heading_err_deg[window]
    if len(heading_t) < 2:
        return {}, ["no paired heading samples inside the metric window"]

    gaps = np.diff(heading_t)
    if len(gaps) and float(gaps.max()) > 2.0:
        reasons.append(f"paired pose gap {float(gaps.max()):.3f} s > 2.0 s")

    required = {"S4": 150}.get(scenario, 300)
    if min_samples is not None:
        required = min_samples
    if len(heading_t) < required:
        reasons.append(f"only {len(heading_t)} paired samples < {required}")

    lat_window = (latency_t >= start) & (latency_t <= end)
    latency_t, latency_ms = latency_t[lat_window], latency_ms[lat_window]
    if len(latency_ms) < 2:
        reasons.append("no output-age samples inside the metric window")
        drift = None
    else:
        lat_order = np.argsort(latency_t)
        latency_t, latency_ms = latency_t[lat_order], latency_ms[lat_order]
        duration = end - start
        if duration >= 60.0:
            first = latency_t <= start + 30.0
            last = latency_t >= end - 30.0
        else:
            first = latency_t <= start + 0.2 * duration
            last = latency_t >= end - 0.2 * duration
        drift = None
        if first.sum() >= 2 and last.sum() >= 2:
            drift = p95(latency_ms[last]) - p95(latency_ms[first])
        else:
            reasons.append("too few output-age samples for the latency-drift windows")

    metrics = {
        "heading_rmse_deg": float(np.sqrt(np.mean(heading_err_deg ** 2))),
        "heading_p95_deg": p95(np.abs(heading_err_deg)),
        "heading_max_deg": float(np.max(np.abs(heading_err_deg))),
        "latency_p95_ms": p95(latency_ms) if len(latency_ms) else None,
        "latency_max_ms": float(np.max(latency_ms)) if len(latency_ms) else None,
        "latency_drift_ms": None if drift is None else float(drift),
        "heading_samples": int(len(heading_t)),
        "latency_samples": int(len(latency_ms)),
        "metric_window_start_s": start,
        "metric_window_end_s": end,
        "metric_window_duration_s": end - start,
    }
    return metrics, reasons


# --------------------------------------------------------------- bag access


def make_typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    if STATUS_MSG.exists():
        msg_text = STATUS_MSG.read_text(encoding="utf-8")
        types = get_types_from_msg(
            msg_text, "brightdrive_interfaces/msg/LocalizationStatus"
        )
        typestore.register(types)
    return typestore


def read_estimator_bag(bag_dir, typestore, base_frame, hero_frame):
    from rosbags.rosbag2 import Reader

    out = {
        "pose_stamps": [],
        "pose_xy": [],
        "pose_yaw": [],
        "pose_bag_ns": [],
        "status_stamps": [],
        "status_latency_ms": [],
        "clock_bag_ns": [],
        "clock_sim_s": [],
        "gnss_stamps": [],
        "gnss_lla": [],
        "gnss_status": [],
        "lever": None,
    }
    with Reader(bag_dir) as reader:
        for connection, timestamp, raw in reader.messages():
            topic = connection.topic
            if topic not in (
                POSE_TOPIC, STATUS_TOPIC, CLOCK_TOPIC, GNSS_TOPIC, TF_STATIC_TOPIC
            ):
                continue
            try:
                msg = typestore.deserialize_cdr(raw, connection.msgtype)
            except Exception:
                continue

            if topic == POSE_TOPIC:
                pose = msg.pose.pose
                out["pose_stamps"].append(stamp_seconds(msg.header.stamp))
                out["pose_xy"].append((pose.position.x, pose.position.y))
                out["pose_yaw"].append(yaw_from_quat(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w))
                out["pose_bag_ns"].append(timestamp)
            elif topic == CLOCK_TOPIC:
                out["clock_bag_ns"].append(timestamp)
                out["clock_sim_s"].append(stamp_seconds(msg.clock))
            elif topic == STATUS_TOPIC:
                try:
                    latency_ms = float(msg.latency_ms)
                except AttributeError:
                    # The #18 readiness stub publishes std_msgs/String here.
                    continue
                out["status_stamps"].append(stamp_seconds(msg.header.stamp))
                out["status_latency_ms"].append(latency_ms)
            elif topic == GNSS_TOPIC:
                out["gnss_stamps"].append(stamp_seconds(msg.header.stamp))
                out["gnss_lla"].append((msg.latitude, msg.longitude))
                out["gnss_status"].append(int(msg.status.status))
            elif topic == TF_STATIC_TOPIC:
                for transform in msg.transforms:
                    if (transform.header.frame_id == base_frame
                            and transform.child_frame_id == hero_frame):
                        translation = transform.transform.translation
                        out["lever"] = [float(translation.x), float(translation.y)]
    for key in ("pose_xy", "gnss_lla"):
        out[key] = np.asarray(out[key], dtype=np.float64)
    for key in ("pose_stamps", "pose_yaw", "status_stamps", "status_latency_ms",
                "clock_bag_ns", "clock_sim_s", "gnss_stamps", "gnss_status"):
        out[key] = np.asarray(out[key], dtype=np.float64)
    return out


def read_gt_bag(bag_dir, typestore):
    from rosbags.rosbag2 import Reader

    stamps, xy, yaw = [], [], []
    with Reader(bag_dir) as reader:
        for connection, _, raw in reader.messages():
            if connection.topic != GT_TOPIC:
                continue
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            pose = msg.pose.pose
            stamps.append(stamp_seconds(msg.header.stamp))
            xy.append((pose.position.x, pose.position.y))
            yaw.append(yaw_from_quat(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w))
    return (np.asarray(stamps, dtype=np.float64),
            np.asarray(xy, dtype=np.float64),
            np.asarray(yaw, dtype=np.float64))


# ------------------------------------------------------- GT frame handling


def load_map_metadata(map_dir):
    map_eval = json.loads((Path(map_dir) / "map_eval.yaml").read_text())
    map_yaml = {}
    map_path = Path(map_dir) / "map.yaml"
    if map_path.exists():
        map_yaml = json.loads(map_path.read_text())
    return map_eval, map_yaml


def gt_base_map(xy_raw, yaw_raw, map_eval, lever):
    """Raw GT hero poses in the ROS world -> (map base_link xy, map yaw)."""
    t_mc = np.asarray(map_eval["T_map_carla"], dtype=np.float64).reshape(4, 4)
    yaw_offset = math.atan2(t_mc[1, 0], t_mc[0, 0])
    semantics = map_eval.get("transform_semantics")
    base_hero_x = float(map_eval.get("base_hero", [lever[0], lever[1], 0.3])[0])
    lever = np.asarray(lever, dtype=np.float64)

    if semantics == "ros_base_link_to_map_base_link":
        cos_yaw, sin_yaw = np.cos(yaw_raw), np.sin(yaw_raw)
        offset = np.column_stack([
            cos_yaw * lever[0] - sin_yaw * lever[1],
            sin_yaw * lever[0] + cos_yaw * lever[1],
        ])
        base_raw = xy_raw - offset
        base_map = (t_mc[:2, :2] @ base_raw.T).T + t_mc[:2, 3]
        return base_map, wrap(yaw_raw + yaw_offset)

    yaw0 = wrap(yaw_raw[0] + yaw_offset)
    t_legacy = t_mc.copy()
    t_legacy[:3, 3] += np.r_[
        base_hero_x * math.cos(yaw0),
        base_hero_x * math.sin(yaw0),
        0.0,
    ]
    hero = (t_legacy[:2, :2] @ xy_raw.T).T + t_legacy[:2, 3]
    yaw_map = wrap(yaw_raw + yaw_offset)
    offset = np.column_stack([
        np.cos(yaw_map) * lever[0] - np.sin(yaw_map) * lever[1],
        np.sin(yaw_map) * lever[0] + np.cos(yaw_map) * lever[1],
    ])
    base_map = hero - offset
    return base_map, yaw_map


def lift_to_hero(xy, yaw, lever):
    offset = np.column_stack([
        np.cos(yaw) * lever[0] - np.sin(yaw) * lever[1],
        np.sin(yaw) * lever[0] + np.cos(yaw) * lever[1],
    ])
    return xy + offset


def gnss_enu(est, map_yaml):
    anchor = map_yaml.get("anchor", {})
    lat0 = anchor.get("lat_deg")
    lon0 = anchor.get("lon_deg")
    if lat0 is None or lon0 is None or not len(est["gnss_lla"]):
        return np.empty((0, 3))
    radius = 6378137.0
    lla = est["gnss_lla"]
    valid = est["gnss_status"] >= 0
    east = np.radians(lla[valid, 1] - lon0) * radius * math.cos(math.radians(lat0))
    north = np.radians(lla[valid, 0] - lat0) * radius
    return np.column_stack([est["gnss_stamps"][valid], east, north])


def resolve_gt_convention(gt_stamps, xy_gt, yaw_gt, map_eval, map_yaml, est, lever):
    """Pick the CARLA-world y convention by GNSS antenna agreement."""
    gnss = gnss_enu(est, map_yaml)
    if not len(gnss):
        return xy_gt, yaw_gt, "as-is (no GNSS to disambiguate)"

    near_gnss, _ = nearest_pairs(gt_stamps, gnss[:, 0])
    gn_xy = gnss[near_gnss, 1:]
    candidates = {
        "as-is": (xy_gt.copy(), yaw_gt.copy()),
        "y-flip": (xy_gt * np.array([1.0, -1.0]), -yaw_gt),
    }
    errors = {}
    base_gnss_x = float(map_eval.get("base_gnss", [lever[0] + 1.0, 0.0, 2.3])[0])
    for name, (xy_raw, yaw_raw) in candidates.items():
        base_map, yaw_map = gt_base_map(xy_raw, yaw_raw, map_eval, lever)
        antenna = base_map + np.column_stack(
            [np.cos(yaw_map), np.sin(yaw_map)]
        ) * base_gnss_x
        errors[name] = float(np.mean(np.linalg.norm(antenna - gn_xy, axis=1)))
    best = min(errors, key=errors.get)
    return candidates[best][0], candidates[best][1], best


# --------------------------------------------------------------- evidence


def write_summary(path, payload):
    metrics = payload["metrics"]
    lines = [
        "# Localization KPI summary",
        "",
        f"- Scenario: `{payload.get('scenario')}`",
        f"- Map: `{payload.get('map_id')}`",
        f"- Valid: `{payload['valid']}`",
        f"- Verdict: **{'PASS' if payload['passed'] else 'FAIL'}**",
        f"- Metric window: {metrics.get('metric_window_start_s', 0):.2f} s "
        f"to {metrics.get('metric_window_end_s', 0):.2f} s "
        f"({metrics.get('metric_window_duration_s', 0):.2f} s)",
        "",
        "| Metric | Value | Threshold | Result |",
        "|---|---:|---:|---|",
    ]
    for name, gate in payload["gates"].items():
        value = gate["value"]
        value_text = "missing" if value is None else f"{value:.6g}"
        lines.append(
            f"| {name.split('/')[-1]} | {value_text} | "
            f"{gate['comparison']} {gate['threshold']:.6g} | "
            f"{'PASS' if gate['passed'] else 'FAIL'} |"
        )
    if payload.get("invalid_reason"):
        lines += ["", f"Invalid run: {payload['invalid_reason']}"]
    lines += ["", f"Output age source: {payload.get('latency_source')}."]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(path, heading_t, heading_err, latency_t, latency_ms):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["kind", "sim_stamp_s", "value"])
        for stamp, value in zip(heading_t, heading_err):
            writer.writerow(["heading_error_deg", f"{stamp:.9f}", f"{value:.6f}"])
        for stamp, value in zip(latency_t, latency_ms):
            writer.writerow(["output_age_ms", f"{stamp:.9f}", f"{value:.6f}"])


def write_plot(path, heading_t, heading_err, latency_t, latency_ms,
               start, end, scenario):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), dpi=120, sharex=True)
    axes[0].plot(heading_t, heading_err, color="#1f77b4", linewidth=1)
    axes[0].axhline(
        kpi_gate.HEADING_GATES.get(scenario, kpi_gate.HEADING_GATES["S1"])
        ["heading_rmse_deg"],
        color="#ff7f0e", linestyle="--", label="RMSE gate",
    )
    axes[0].set_ylabel("heading error [deg]")
    axes[0].legend()
    axes[0].set_title(f"{scenario} heading error and published-pose message age")
    axes[1].plot(latency_t, latency_ms, color="#2ca02c", linewidth=1)
    axes[1].axhline(
        kpi_gate.LATENCY_GATES["latency_p95_ms"],
        color="#d62728", linestyle="--", label="p95 gate",
    )
    axes[1].axhline(
        kpi_gate.LATENCY_GATES["latency_max_ms"],
        color="#d62728", linestyle=":", label="max gate",
    )
    axes[1].set_ylabel("output age [ms]")
    axes[1].set_xlabel("simulation time [s]")
    axes[1].legend()
    for axis in axes:
        axis.axvline(start, color="#777777", linewidth=0.8)
        axis.axvline(end, color="#777777", linewidth=0.8)
        axis.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return True


# --------------------------------------------------------------- evaluation


def evaluate(estimator_bag, gt_bag, map_dir, args):
    typestore = make_typestore()
    est = read_estimator_bag(estimator_bag, typestore, args.base_frame, args.hero_frame)
    gt_stamps, gt_xy, gt_yaw = read_gt_bag(gt_bag, typestore)
    map_eval, map_yaml = load_map_metadata(map_dir)

    if not len(est["pose_stamps"]):
        raise SystemExit(f"no {POSE_TOPIC} in {estimator_bag}")
    if not len(gt_stamps):
        raise SystemExit(f"no {GT_TOPIC} in {gt_bag}")

    lever = est["lever"]
    if lever is None:
        lever = [float(map_eval.get("base_hero", [1.394, 0.0, 0.3])[0]),
                 float(map_eval.get("base_hero", [1.394, 0.0, 0.3])[1])]
    gt_xy, gt_yaw, convention = resolve_gt_convention(
        gt_stamps, gt_xy, gt_yaw, map_eval, map_yaml, est, lever
    )
    gt_base, gt_yaw_map = gt_base_map(gt_xy, gt_yaw, map_eval, lever)
    gt_hero = lift_to_hero(gt_base, gt_yaw_map, lever)

    est_stamps = est["pose_stamps"]
    est_xy = est["pose_xy"]
    est_yaw = est["pose_yaw"]
    est_hero = lift_to_hero(est_xy, est_yaw, lever)

    near, dt = nearest_pairs(gt_stamps, est_stamps)
    keep = dt <= args.max_dt
    if not keep.any():
        raise SystemExit(
            f"no estimator poses within {args.max_dt:.3f} s of GT; check bags"
        )
    paired_t = gt_stamps[keep]
    heading_err = np.degrees(np.abs(wrap(est_yaw[near[keep]] - gt_yaw_map[keep])))
    # Position is not gated here (#11 owns ATE); it is reported for context.
    position_err = np.linalg.norm(est_hero[near[keep]] - gt_hero[keep], axis=1)

    sim_bag = clock_to_sim(est["pose_bag_ns"], est["clock_bag_ns"], est["clock_sim_s"])
    latency_t = est_stamps[~np.isnan(sim_bag)]
    latency_ms = (sim_bag[~np.isnan(sim_bag)] - latency_t) * 1000.0

    metrics, reasons = score(
        paired_t, heading_err, latency_t, latency_ms,
        args.scenario, args.settle_s, args.min_samples,
    )
    if len(paired_t):
        paired_fraction = len(paired_t) / len(gt_stamps)
    else:
        paired_fraction = 0.0
    if paired_fraction < args.min_paired_fraction:
        reasons.append(
            f"paired completeness {paired_fraction:.3f} < {args.min_paired_fraction:.3f}"
        )
    if not len(est["clock_sim_s"]):
        reasons = ["no /clock in the estimator bag; cannot convert output age to sim time"]

    if metrics:
        metrics.update({
            "paired_fraction": float(paired_fraction),
            "position_rmse_m_report_only": float(np.sqrt(np.mean(position_err ** 2))),
            "position_p95_m_report_only": p95(position_err),
            "gt_convention": convention,
            "status_latency_p95_ms": (
                p95(est["status_latency_ms"]) if len(est["status_latency_ms"]) else None
            ),
        })

    valid = not reasons and bool(metrics) and all(
        metrics.get(key) is not None for key in METRIC_KEYS
    )
    payload = {
        "schema_version": 1,
        "kind": "run_metrics",
        "scenario": args.scenario,
        "map_id": map_eval.get("map_id"),
        "valid": bool(valid),
        "invalid_reason": "; ".join(reasons),
        "latency_source": "sim-time bag reception minus /odometry/filtered header stamp",
        "metrics": metrics,
    }
    checks, failures = kpi_gate.gate_metrics(payload)
    payload["gates"] = {
        check["name"]: {
            "value": check["value"],
            "threshold": check["threshold"],
            "comparison": check["comparison"],
            "passed": check["passed"],
        }
        for check in checks
    }
    payload["failures"] = failures
    payload["passed"] = not failures

    out_dir = Path(args.out_dir) if args.out_dir else Path(estimator_bag).resolve().parent
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "metrics.json").write_text(json.dumps(payload, indent=2) + "\n")
    write_summary(out_dir / "metrics_summary.md", payload)
    write_csv(out_dir / "metrics.csv", paired_t, heading_err, latency_t, latency_ms)
    if metrics:
        write_plot(
            out_dir / "kpi_heading_latency.png",
            paired_t, heading_err, latency_t, latency_ms,
            metrics["metric_window_start_s"], metrics["metric_window_end_s"],
            args.scenario,
        )

    for check in checks:
        value = "missing" if check["value"] is None else f"{check['value']:.6g}"
        print(
            f"KPI CHECK {check['name']}={value} "
            f"{check['comparison']} {check['threshold']:.6g} "
            f"{'PASS' if check['passed'] else 'FAIL'}"
        )
    for failure in failures:
        print("KPI FAIL:", failure, file=sys.stderr)
    print(
        f"KPI_EVIDENCE {out_dir / 'metrics.json'} scenario={args.scenario} "
        f"valid={valid} passed={payload['passed']}"
    )
    return bool(payload["passed"])


# ------------------------------------------------- synthetic bag round trip
#
# The metric maths above is exercised on hand-built arrays.  The path that
# turns a recorded bag into those arrays -- CDR deserialisation, the
# base_link <-> hero lever, T_map_carla, the /clock -> sim-time conversion --
# only fails on real bags, so it gets one end-to-end round trip on a
# synthetic pair whose injected error is known exactly.

LEVER = (1.394, 0.0, 0.30)  # base_link -> hero, from bringup/urdf
BASE_GNSS_X = 2.394         # base_link -> GNSS antenna, from map_eval.yaml
ANCHOR = (52.370216, 4.895168)

_MSG = {}


def _msg(name):
    """Message class for ``name``, resolved on first use (rosbags is optional)."""
    if name not in _MSG:
        _MSG[name] = make_typestore().types[name]
    return _MSG[name]


def _stamp(stamp_s):
    return _msg("builtin_interfaces/msg/Time")(
        sec=int(stamp_s), nanosec=int(round((stamp_s % 1.0) * 1e9))
    )


def _header(stamp_s, frame_id):
    return _msg("std_msgs/msg/Header")(stamp=_stamp(stamp_s), frame_id=frame_id)


def _yaw_quat(yaw):
    return _msg("geometry_msgs/msg/Quaternion")(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
    )


def _pose(x, y, yaw):
    return _msg("geometry_msgs/msg/PoseWithCovariance")(
        pose=_msg("geometry_msgs/msg/Pose")(
            position=_msg("geometry_msgs/msg/Point")(x=x, y=y, z=0.0),
            orientation=_yaw_quat(yaw),
        ),
        covariance=np.zeros(36),
    )


def _odometry(stamp_s, x, y, yaw, frame_id, child_frame_id):
    return _msg("nav_msgs/msg/Odometry")(
        header=_header(stamp_s, frame_id),
        child_frame_id=child_frame_id,
        pose=_pose(x, y, yaw),
        twist=_msg("geometry_msgs/msg/TwistWithCovariance")(
            twist=_msg("geometry_msgs/msg/Twist")(
                linear=_msg("geometry_msgs/msg/Vector3")(x=0.0, y=0.0, z=0.0),
                angular=_msg("geometry_msgs/msg/Vector3")(x=0.0, y=0.0, z=0.0),
            ),
            covariance=np.zeros(36),
        ),
    )


def _navsat(stamp_s, latitude, longitude):
    return _msg("sensor_msgs/msg/NavSatFix")(
        header=_header(stamp_s, "hero/gnss"),
        status=_msg("sensor_msgs/msg/NavSatStatus")(status=0, service=1),
        latitude=latitude,
        longitude=longitude,
        altitude=3.7,
        position_covariance=np.zeros(9),
        position_covariance_type=0,
    )


def _tf_static(lever):
    return _msg("tf2_msgs/msg/TFMessage")(
        transforms=[
            _msg("geometry_msgs/msg/TransformStamped")(
                header=_header(0.0, "base_link"),
                child_frame_id="hero",
                transform=_msg("geometry_msgs/msg/Transform")(
                    translation=_msg("geometry_msgs/msg/Vector3")(
                        x=lever[0], y=lever[1], z=lever[2]
                    ),
                    rotation=_yaw_quat(0.0),
                ),
            )
        ]
    )


def _lla(anchor, east, north):
    radius = 6378137.0
    lat0, lon0 = anchor
    return (
        lat0 + math.degrees(north / radius),
        lon0 + math.degrees(east / (radius * math.cos(math.radians(lat0)))),
    )


def synthetic_run(root, yaw_offset_deg=1.5, lag_ms=120.0, with_clock=True,
                  hz=10.0, duration=40.0):
    # ``lag_ms``: float, or a callable ``lag_ms(stamp, duration)`` for a ramp.
    """Write a synthetic estimator bag, GT bag and map bundle; run the evaluator.

    The estimator drives +x at ``hz`` with a constant yaw offset and a bag
    reception lag of ``lag_ms`` (a callable ``lag_ms(stamp, duration)`` gives a
    ramp); GT is the same path with no offset.  ``/clock`` is recorded with
    wall == sim, so ``clock_to_sim`` recovers the injected lag exactly.  Returns
    the parsed ``metrics.json`` payload.
    """
    from rosbags.rosbag2 import Writer

    typestore = make_typestore()
    root.mkdir(parents=True, exist_ok=True)
    sim = np.arange(0.0, duration, 1.0 / hz)
    yaw = math.radians(yaw_offset_deg)
    map_dir = root / "map"
    map_dir.mkdir(exist_ok=True)
    (map_dir / "map_eval.yaml").write_text(json.dumps({
        "schema_version": 1, "map_id": "synthetic",
        "transform_semantics": "ros_base_link_to_map_base_link",
        "T_map_carla": np.eye(4).reshape(-1).tolist(),
        "base_hero": list(LEVER),
        "base_gnss": [BASE_GNSS_X, 0.0, 2.3],
    }))
    (map_dir / "map.yaml").write_text(json.dumps({
        "schema_version": 1, "map_id": "synthetic", "map_frame": "map",
        "anchor": {"lat_deg": ANCHOR[0], "lon_deg": ANCHOR[1], "alt_m": 3.7},
    }))

    with Writer(root / "est", version=9) as writer:
        pose_c = writer.add_connection("/odometry/filtered", "nav_msgs/msg/Odometry",
                                       typestore=typestore)
        clock_c = writer.add_connection("/clock", "rosgraph_msgs/msg/Clock",
                                        typestore=typestore)
        gnss_c = writer.add_connection("/carla/hero/gnss", "sensor_msgs/msg/NavSatFix",
                                       typestore=typestore)
        tf_c = writer.add_connection("/tf_static", "tf2_msgs/msg/TFMessage",
                                     typestore=typestore)
        for stamp in sim:
            x = 3.0 * stamp
            lag = lag_ms(stamp, duration) if callable(lag_ms) else lag_ms
            writer.write(pose_c, int(round(stamp * 1e9 + lag * 1e6)),
                         typestore.serialize_cdr(
                             _odometry(stamp, x, 0.0, yaw, "map", "base_link"),
                             "nav_msgs/msg/Odometry"))
            if with_clock:
                writer.write(clock_c, int(round(stamp * 1e9)),
                             typestore.serialize_cdr(
                                 _msg("rosgraph_msgs/msg/Clock")(clock=_stamp(stamp)),
                                 "rosgraph_msgs/msg/Clock"))
            lat, lon = _lla(ANCHOR, x + BASE_GNSS_X, 0.0)
            writer.write(gnss_c, int(round(stamp * 1e9)),
                         typestore.serialize_cdr(
                             _navsat(stamp, lat, lon), "sensor_msgs/msg/NavSatFix"))
        writer.write(tf_c, 0, typestore.serialize_cdr(
            _tf_static(LEVER), "tf2_msgs/msg/TFMessage"))

    with Writer(root / "gt", version=9) as writer:
        gt_c = writer.add_connection("/carla/hero/odometry", "nav_msgs/msg/Odometry",
                                     typestore=typestore)
        clock_c = writer.add_connection("/clock", "rosgraph_msgs/msg/Clock",
                                        typestore=typestore)
        for stamp in sim:
            writer.write(gt_c, int(round(stamp * 1e9)),
                         typestore.serialize_cdr(
                             _odometry(stamp, 3.0 * stamp + LEVER[0], 0.0, 0.0,
                                       "hero", "hero"),
                             "nav_msgs/msg/Odometry"))
            writer.write(clock_c, int(round(stamp * 1e9)),
                         typestore.serialize_cdr(
                             _msg("rosgraph_msgs/msg/Clock")(clock=_stamp(stamp)),
                             "rosgraph_msgs/msg/Clock"))

    args = argparse.Namespace(
        scenario="S1", max_dt=0.15, settle_s=2.0, min_samples=None,
        min_paired_fraction=0.9, base_frame="base_link", hero_frame="hero",
        out_dir=str(root / "ev"),
    )
    evaluate(str(root / "est"), str(root / "gt"), str(map_dir), args)
    return json.loads((root / "ev" / "metrics.json").read_text())


def self_test():
    assert abs(wrap(math.radians(2.0))) - math.radians(2.0) < 1e-9
    assert abs(clock_to_sim([15, 25], [10, 20, 30], [1.0, 2.0, 3.0])[0] - 1.5) < 1e-9
    assert math.isnan(clock_to_sim([5], [10, 20], [1.0, 2.0])[0])

    t = np.arange(0.0, 40.0, 0.1)
    heading = np.ones_like(t) * 1.0
    latency = np.ones_like(t) * 100.0
    metrics, reasons = score(t, heading, t, latency, "S1", settle_s=2.0)
    assert not reasons, reasons
    assert abs(metrics["latency_drift_ms"]) < 1e-9, metrics
    checks, failures = kpi_gate.gate_metrics({
        "schema_version": 1, "kind": "run_metrics", "scenario": "S1",
        "valid": True, "metrics": metrics,
    })
    assert not failures, failures

    latency_drift = latency.copy()
    latency_drift[t < 20.0] = 100.0
    latency_drift[t >= 20.0] = 300.0
    metrics, reasons = score(t, heading, t, latency_drift, "S1", settle_s=2.0)
    assert not reasons, reasons
    assert metrics["latency_drift_ms"] > 50.0, metrics
    _, failures = kpi_gate.gate_metrics({
        "schema_version": 1, "kind": "run_metrics", "scenario": "S1",
        "valid": True, "metrics": metrics,
    })
    assert any("latency_drift_ms" in failure for failure in failures), failures

    metrics, reasons = score([], [], [], [], "S1")
    assert metrics == {} and reasons, (metrics, reasons)

    with tempfile.TemporaryDirectory() as tmp:
        # The injected yaw offset and bag-reception lag must come back exactly.
        payload = synthetic_run(Path(tmp), yaw_offset_deg=1.5, lag_ms=120.0)
        metrics = payload["metrics"]
        assert payload["valid"] and payload["passed"], payload["invalid_reason"]
        assert abs(metrics["heading_rmse_deg"] - 1.5) < 1e-6, metrics
        assert abs(metrics["latency_p95_ms"] - 120.0) < 1e-6, metrics
        assert abs(metrics["latency_drift_ms"]) < 1e-6, metrics
        assert metrics["gt_convention"] == "as-is", metrics["gt_convention"]

        # A yaw offset past the S1 RMSE gate must fail, not pass quietly.
        payload = synthetic_run(Path(tmp) / "over", yaw_offset_deg=2.5, lag_ms=120.0)
        assert not payload["passed"], payload["metrics"]
        assert any("heading_rmse_deg" in f for f in payload["failures"]), payload

        # An accumulating output age must trip the drift gate.
        payload = synthetic_run(
            Path(tmp) / "ramp", yaw_offset_deg=1.0,
            lag_ms=lambda stamp, duration: 100.0 + 100.0 * stamp / duration)
        assert not payload["passed"], payload["metrics"]
        assert any("latency_drift_ms" in f for f in payload["failures"]), payload

        # No /clock means output age is unmeasurable: invalid, never a pass.
        payload = synthetic_run(Path(tmp) / "noclock", yaw_offset_deg=1.0,
                                lag_ms=120.0, with_clock=False)
        assert not payload["valid"], payload
        assert not payload["passed"], payload
    print("KPI_SELF_TEST ok")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("estimator_bag", nargs="?")
    parser.add_argument("gt_bag", nargs="?")
    parser.add_argument("map_dir", nargs="?")
    parser.add_argument("--scenario", choices=sorted(kpi_gate.HEADING_GATES), default="S1")
    parser.add_argument("--out-dir")
    parser.add_argument("--max-dt", type=float, default=0.15)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--min-paired-fraction", type=float, default=0.9)
    parser.add_argument("--min-samples", type=int)
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--hero-frame", default="hero")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.estimator_bag or not args.gt_bag or not args.map_dir:
        parser.error("need <estimator_bag> <gt_bag> <map_dir> (or --self-test)")
    if not evaluate(args.estimator_bag, args.gt_bag, args.map_dir, args):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
