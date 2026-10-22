#!/usr/bin/env python3
"""Probe the Town05 road graph and report candidate mapping loops."""

import argparse
import math
import sys
import time

import carla

sys.path.insert(0, "/opt/carla/PythonAPI/carla")
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402


def wait_hero(world, role, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role:
                return actor
        time.sleep(0.5)
    raise RuntimeError(f"no hero vehicle with role {role}")


def route_length(route):
    return sum(
        route[i][0].transform.location.distance(route[i + 1][0].transform.location)
        for i in range(len(route) - 1)
    )


def route_stats(route):
    turns = 0
    junctions = 0
    in_junction = False
    for i in range(2, len(route)):
        a = route[i - 2][0].transform.location
        b = route[i - 1][0].transform.location
        c = route[i][0].transform.location
        h1 = math.atan2(b.y - a.y, b.x - a.x)
        h2 = math.atan2(c.y - b.y, c.x - b.x)
        delta = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        if delta > math.radians(40.0):
            turns += 1
        if route[i][0].is_junction and not in_junction:
            junctions += 1
        in_junction = route[i][0].is_junction
    return turns, junctions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero")
    parser.add_argument("--speed-kmh", type=float, default=30.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    wait_hero(world, args.role, 60.0)
    wmap = world.get_map()
    spawns = wmap.get_spawn_points()
    print(f"map={wmap.name} spawns={len(spawns)}")

    start = spawns[0]
    print(
        "spawn[0] "
        f"x={start.location.x:.2f} y={start.location.y:.2f} z={start.location.z:.2f} "
        f"yaw={start.rotation.yaw:.2f}"
    )
    for i, sp in enumerate(spawns):
        d = start.location.distance(sp.location)
        print(
            f"spawn[{i}] x={sp.location.x:.1f} y={sp.location.y:.1f} "
            f"yaw={sp.rotation.yaw:.0f} dist={d:.0f}"
        )

    xs = [s.location.x for s in spawns]
    ys = [s.location.y for s in spawns]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    print(f"centroid=({cx:.1f},{cy:.1f}) extent=({max(xs)-min(xs):.0f},{max(ys)-min(ys):.0f})")

    quadrants = {}
    for i, sp in enumerate(spawns):
        if i == 0:
            continue
        q = (sp.location.x >= cx, sp.location.y >= cy)
        d = start.location.distance(sp.location)
        if q not in quadrants or d > quadrants[q][0]:
            quadrants[q] = (d, i)
    picks = [i for _, i in quadrants.values()]
    picks.sort(key=lambda i: math.atan2(spawns[i].location.y - cy, spawns[i].location.x - cx))
    loop = [0] + picks + [0]
    print(f"loop_spawns={loop}")

    grp = GlobalRoutePlanner(wmap, 1.0)
    total = 0.0
    for a, b in zip(loop, loop[1:]):
        route = grp.trace_route(spawns[a].location, spawns[b].location)
        length = route_length(route)
        turns, junctions = route_stats(route)
        total += length
        print(
            f"leg {a}->{b}: points={len(route)} length={length:.1f}m "
            f"turns={turns} junctions={junctions}"
        )
    print(
        f"loop_total={total:.1f}m at {args.speed_kmh:.0f} km/h "
        f"= {total / (args.speed_kmh / 3.6) / 60.0:.1f} min"
    )


if __name__ == "__main__":
    main()
