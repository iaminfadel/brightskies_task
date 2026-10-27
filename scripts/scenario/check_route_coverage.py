#!/usr/bin/env python3
"""Fail fast when a scenario route leaves saved-map coverage (#37).

Every scenario route must stay inside map coverage: the part of the saved
map within `--coverage-m` (default 20 m) of the mapping trajectory. This
tool parses the route waypoints (and any teleport positions) out of an
OpenSCENARIO file, converts them into the map frame, and measures the
distance of every waypoint to the mapping trajectory. A route that leaves
coverage exits non-zero *before* any expensive runtime comes up -- a route
outside coverage is a scenario-design failure, not a runtime problem.

Coordinate chain (all verified against the bridge and the map contract):

    OSC WorldPosition  --y-flip-->  CARLA (x, -y as SR flips y back)
    CARLA              --bridge-->  ROS world (x, -y)
    ROS world          --T_map_carla-->  map frame (trajectory.tum)

SR's OpenScenarioParser flips the y of a standard-OSC WorldPosition before
using it as a CARLA coordinate, and the bridge's ROS conversion is
(x_ros, y_ros) = (x_carla, -y_carla); the two flips cancel, so an OSC
waypoint (x, y) lands at ROS-world (x, y) and then T_map_carla applies.

The committed maps/<map-id>/coverage.tum is the mapping trajectory
downsampled to ~2 m spacing; the full-resolution trajectory.tum in the VM
bundle is used when present. Both are in the map frame.
"""
import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]


def osc_world_positions(scenario_file):
    """All (x, y, z) OSC WorldPositions used as routes or teleports."""
    root = ET.parse(scenario_file).getroot()
    positions = []
    for waypoint in root.iter("Waypoint"):
        position = waypoint.find("Position")
        if position is not None and position.find("WorldPosition") is not None:
            positions.append(("route", _read_world_position(position)))
    for teleport in root.iter("TeleportAction"):
        position = teleport.find("Position")
        if position is not None and position.find("WorldPosition") is not None:
            positions.append(("teleport", _read_world_position(position)))
    if not positions:
        raise SystemExit(f"no route Waypoints or Teleport WorldPositions in {scenario_file}")
    return positions


def _read_world_position(position):
    world = position.find("WorldPosition")
    return (
        float(world.attrib.get("x", 0.0)),
        float(world.attrib.get("y", 0.0)),
        float(world.attrib.get("z", 0.0)),
    )


def osc_to_map(points, t_map_carla):
    """OSC WorldPositions -> map-frame xy (see module docstring)."""
    t = np.asarray(t_map_carla, dtype=np.float64).reshape(4, 4)
    ros = np.asarray(points, dtype=np.float64)  # y-flips cancel; z rides along
    ros_h = np.column_stack([ros[:, 0], ros[:, 1], ros[:, 2], np.ones(len(ros))])
    mapped = (t @ ros_h.T).T[:, :2]
    return mapped


def load_t_map_carla(map_eval_file):
    doc = json.loads(Path(map_eval_file).read_text())
    semantics = doc.get("transform_semantics")
    if semantics != "ros_base_link_to_map_base_link":
        raise SystemExit(
            f"unsupported T_map_carla semantics {semantics!r} in {map_eval_file}"
        )
    return np.asarray(doc["T_map_carla"], dtype=np.float64)


def load_trajectory(tum_file):
    points = []
    with open(tum_file, "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) >= 3:
                points.append((float(parts[1]), float(parts[2])))
    if not points:
        raise SystemExit(f"no trajectory poses in {tum_file}")
    return np.asarray(points)


def coverage_distances(route_map_xy, trajectory_xy):
    """Nearest-trajectory distance per route point (metres)."""
    trajectory = trajectory_xy[:, None, :]  # (T, 1, 2)
    deltas = route_map_xy[None, :, :] - trajectory  # (T, R, 2)
    return np.sqrt((deltas ** 2).sum(axis=2)).min(axis=0)


def run_preflight(scenario_file, map_eval_file, trajectory_file, coverage_m=20.0):
    """Library entry for run_scenario.py: returns the coverage summary."""
    t_map_carla = load_t_map_carla(map_eval_file)
    trajectory = load_trajectory(trajectory_file)
    positions = osc_world_positions(scenario_file)
    route_map = osc_to_map([p for _, p in positions], t_map_carla)
    distances = coverage_distances(route_map, trajectory)
    worst_idx = int(np.argmax(distances))
    return {
        "scenario_file": str(scenario_file),
        "map_eval": str(map_eval_file),
        "trajectory": str(trajectory_file),
        "coverage_m": coverage_m,
        "waypoints": len(distances),
        "max_distance_m": float(distances[worst_idx]),
        "mean_distance_m": float(distances.mean()),
        "worst_waypoint": {"kind": positions[worst_idx][0],
                           "osc": [round(v, 3) for v in positions[worst_idx][1]],
                           "map": [round(float(v), 3) for v in route_map[worst_idx]]},
        "inside_coverage": bool(distances.max() <= coverage_m),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario_file", help="OpenSCENARIO (.xosc) file")
    parser.add_argument("--map-eval", default=str(REPO / "docs/results/map/town05-v14-pg/map_eval.yaml"),
                        help="map_eval.yaml with T_map_carla")
    parser.add_argument("--trajectory", default=str(REPO / "maps/town05-v14-pg/coverage.tum"),
                        help="mapping trajectory (TUM) in the map frame")
    parser.add_argument("--coverage-m", type=float, default=20.0,
                        help="coverage envelope around the mapping trajectory")
    parser.add_argument("--json-out", default=None, help="write a summary JSON here")
    args = parser.parse_args()

    t_map_carla = load_t_map_carla(args.map_eval)
    trajectory = load_trajectory(args.trajectory)

    positions = osc_world_positions(args.scenario_file)
    points = [p for _, p in positions]
    route_map = osc_to_map(points, t_map_carla)
    distances = coverage_distances(route_map, trajectory)

    kinds = [kind for kind, _ in positions]
    worst_idx = int(np.argmax(distances))
    summary = {
        "scenario_file": str(args.scenario_file),
        "map_eval": str(args.map_eval),
        "trajectory": str(args.trajectory),
        "coverage_m": args.coverage_m,
        "waypoints": len(distances),
        "max_distance_m": float(distances[worst_idx]),
        "mean_distance_m": float(distances.mean()),
        "worst_waypoint": {"kind": kinds[worst_idx],
                           "osc": [round(v, 3) for v in points[worst_idx]],
                           "map": [round(float(v), 3) for v in route_map[worst_idx]]},
        "inside_coverage": bool(distances.max() <= args.coverage_m),
    }
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(summary, indent=2) + "\n")

    print(
        f"coverage: {len(distances)} waypoints, max {summary['max_distance_m']:.2f} m "
        f"(envelope {args.coverage_m:.0f} m) -> "
        f"{'INSIDE' if summary['inside_coverage'] else 'OUTSIDE'}"
    )
    if not summary["inside_coverage"]:
        worst = summary["worst_waypoint"]
        print(
            f"ROUTE_OUTSIDE_COVERAGE: {worst['kind']} waypoint at map "
            f"({worst['map'][0]:.1f}, {worst['map'][1]:.1f}) is "
            f"{summary['max_distance_m']:.1f} m from the mapping trajectory",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
