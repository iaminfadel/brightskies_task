#!/usr/bin/env python3
"""Dev-time route designer: derive the four scenario routes from the map.

The scenario routes are waypoint subsets of the *actual mapped loop*
(trajectory.tum), so coverage is by construction and the OSC waypoints are
exact CARLA coordinates. This tool converts the mapping trajectory from the
map frame back to CARLA coordinates, finds the locked segments (#10 §1):

  S1/S2  urban segment, low curvature, ~300 m, 30 km/h
  S3     two ~90 degree turns + a >=200 m straight, 50 km/h
  S4     the approach to a signalized intersection on the loop

and (when the CARLA server is reachable) lists the traffic lights close to
the route so S4 can pick one. Output is JSON on stdout; the committed
scenarios/*.xosc are authored from it. Not used at run time -- the run-time
guarantee comes from check_route_coverage.py.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]


def load_t_map_carla(map_eval_file):
    doc = json.loads(Path(map_eval_file).read_text())
    return np.asarray(doc["T_map_carla"], dtype=np.float64).reshape(4, 4)


def load_trajectory(tum_file):
    rows = []
    with open(tum_file, "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) >= 8:
                rows.append([float(v) for v in parts[1:4]])
    return np.asarray(rows)  # tx ty tz per pose, map frame


def map_to_carla(points_map, t_map_carla):
    """Invert the OSC->map chain: map -> ROS world -> CARLA (y flip)."""
    t_inv = np.linalg.inv(t_map_carla)
    hom = np.column_stack([points_map, np.ones(len(points_map))])
    ros = (t_inv @ hom.T).T[:, :3]
    return np.column_stack([ros[:, 0], -ros[:, 1], ros[:, 2]])


def resample(points, spacing):
    """Arc-length resample to ~spacing metres, keeping direction."""
    seg = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(int(total // spacing), 2)
    targets = np.linspace(0.0, total, n)
    out = np.column_stack([
        np.interp(targets, s, points[:, 0]),
        np.interp(targets, s, points[:, 1]),
        np.interp(targets, s, points[:, 2]),
    ])
    return out, s, targets


def headings(points):
    before = np.roll(points[:, :2], 1, axis=0)
    after = np.roll(points[:, :2], -1, axis=0)
    delta = after - before
    return np.arctan2(delta[:, 1], delta[:, 0])


def max_turn(points, idx, window_m, spacing):
    """Max absolute heading change within window_m after idx."""
    count = max(int(window_m / spacing), 2)
    window = headings(points[idx:min(idx + count, len(points))])
    if len(window) < 2:
        return 0.0
    delta = np.diff(np.unwrap(window))
    return float(np.abs(delta).max())


def straight_runs(points, spacing, straight_m, straight_tol_deg):
    """(start_idx, end_idx) spans with no turn above tol over straight_m."""
    spans = []
    count = int(straight_m / spacing)
    i = 0
    while i + count < len(points):
        window = headings(points[i:i + count + 1])
        if len(window) > 2 and np.abs(np.diff(np.unwrap(window))).max() <= math.radians(straight_tol_deg):
            j = i + count
            while j + 1 < len(points) and abs(_turn_at(points, j)) <= math.radians(straight_tol_deg):
                j += 1
            spans.append((i, j))
            i = j
        else:
            i += 1
    return spans


def _turn_at(points, idx):
    if idx <= 0 or idx + 1 >= len(points):
        return 0.0
    delta = headings(points[idx + 1:idx + 2])[0] - headings(points[idx:idx + 1])[0]
    return math.atan2(math.sin(delta), math.cos(delta))


def find_s3(points, spacing):
    """Two ~90 degree turns followed by a >=200 m straight."""
    n = len(points)
    for i in range(n):
        t1 = _abs_turn_over(points, i, spacing, 25.0)
        if not (math.radians(60) <= t1 <= math.radians(120)):
            continue
        for j in range(i + 1, min(i + int(120 / spacing), n)):
            t2 = _abs_turn_over(points, j, spacing, 25.0)
            if not (math.radians(60) <= t2 <= math.radians(120)):
                continue
            straight_end = j + int(200 / spacing)
            if straight_end >= n:
                continue
            window = headings(points[j:straight_end + 1])
            if len(window) > 2 and np.abs(np.diff(np.unwrap(window))).max() <= math.radians(15):
                return i, straight_end
    return None


def _abs_turn_over(points, idx, spacing, window_m):
    count = max(int(window_m / spacing), 2)
    if idx + count >= len(points):
        return 0.0
    window = headings(points[idx:idx + count])
    delta = np.unwrap(window)[-1] - np.unwrap(window)[0]
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def find_urban(points, spacing, length_m=300.0):
    """Low-curvature stretch of length_m that is not the S3 straight."""
    count = int(length_m / spacing)
    best = None
    best_turn = None
    for i in range(0, len(points) - count, 5):
        window = headings(points[i:i + count + 1])
        turn = float(np.abs(np.diff(np.unwrap(window))).sum()) if len(window) > 2 else 9.9
        if best_turn is None or turn < best_turn:
            best_turn = turn
            best = (i, i + count)
    return best


def traffic_lights_near(route, max_dist_m=40.0):
    """CARLA traffic lights within max_dist_m of the route (needs server)."""
    try:
        import carla
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
    except Exception as error:  # noqa: BLE001 - probe only
        return {"error": str(error)}
    lights = []
    for light in world.get_actors().filter("traffic.traffic_light"):
        loc = light.get_location()
        dist = float(np.min(np.linalg.norm(route[:, :2] - np.array([loc.x, loc.y]), axis=1)))
        if dist <= max_dist_m:
            lights.append({
                "id": light.id,
                "carla": [round(loc.x, 2), round(loc.y, 2), round(loc.z, 2)],
                "distance_m": round(dist, 2),
                "pole_index": light.get_pole_index(),
                "road_id": light.get_group_traffic_lights() and None,
            })
            lights[-1].pop("road_id")
    return {"lights": sorted(lights, key=lambda light: light["distance_m"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trajectory", default="/ws/maps/current/town05-v14-pg/trajectory.tum")
    parser.add_argument("--map-eval", default="/ws/maps/builds/town05-v14-pg/map_eval.yaml")
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--with-traffic-lights", action="store_true")
    args = parser.parse_args()

    t_map_carla = load_t_map_carla(args.map_eval)
    traj_map = load_trajectory(args.trajectory)
    carla_points = map_to_carla(traj_map, t_map_carla)

    route, _s, _t = resample(carla_points, args.spacing)
    spacing = args.spacing

    s3 = find_s3(route, spacing)
    urban = find_urban(route, spacing)

    def seg(idx):
        if idx is None:
            return None
        start, end = idx
        return {
            "start_idx": int(start), "end_idx": int(end),
            "length_m": round(float((end - start) * spacing), 1),
            "start_carla": [round(float(v), 2) for v in route[start]],
            "end_carla": [round(float(v), 2) for v in route[end]],
            "start_yaw_deg": round(math.degrees(headings(route)[start]), 1),
        }

    result = {
        "trajectory_points": int(len(traj_map)),
        "loop_length_m": round(float(np.linalg.norm(np.diff(carla_points[:, :2], axis=0), axis=1).sum()), 1),
        "spacing_m": spacing,
        "S3_turns_straight": seg(s3),
        "S1_urban": seg(urban),
        "route_carla_xy": [
            [round(float(x), 2), round(float(y), 2)] for x, y, _z in route[::5]
        ],
    }
    if args.with_traffic_lights:
        result["traffic_lights"] = traffic_lights_near(route)
    json.dump(result, sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    main()
