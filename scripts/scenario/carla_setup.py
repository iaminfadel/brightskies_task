#!/usr/bin/env python3
"""CARLA-side setup steps for a scenario run, in a short-lived process.

The carla Python client keeps background streaming threads that can throw
into std::terminate when the server resets sensor streams (observed when
ScenarioRunner attaches while a client lingers in the parent process). All
client work therefore runs here and the process exits before SR launches;
each subcommand prints one JSON event line for the harness to log.
"""
import argparse
import json
import sys
import time

import carla


def client(timeout=30.0):
    c = carla.Client("127.0.0.1", 2000)
    c.set_timeout(timeout)
    return c


def emit(event, **details):
    print(json.dumps({"event": event, **details}), flush=True)


def find_hero(world):
    for _ in range(20):
        actors = world.get_actors().filter("vehicle.*")
        hero = next((a for a in actors if a.attributes.get("role_name") == "hero"), None)
        if hero is not None:
            return hero
        time.sleep(0.5)
    raise RuntimeError("no hero vehicle in the world")


def destroy_hero(_args):
    world = client().get_world()
    for _ in range(10):
        heroes = [a for a in world.get_actors() if a.attributes.get("role_name") == "hero"]
        if heroes:
            for actor in heroes:
                actor.destroy()
            emit("hero_destroyed", count=len(heroes))
            return
        try:
            world.wait_for_tick(2.0)
        except RuntimeError:
            pass  # sync mode: the bridge owns ticks; a missed tick is harmless
    raise RuntimeError("no hero to destroy")


def route_start_pose(scenario_file):
    """CARLA pose (x, y, z, yaw_deg) at the first route waypoint: the .xosc
    stores standard-OSC coordinates (y = -y_carla)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(scenario_file).getroot()
    points = []
    for waypoint in root.iter("Waypoint"):
        position = waypoint.find("Position")
        world = position.find("WorldPosition") if position is not None else None
        if world is not None:
            points.append((
                float(world.attrib.get("x", 0.0)),
                -float(world.attrib.get("y", 0.0)),
                float(world.attrib.get("z", 0.0)),
            ))
    if len(points) < 2:
        raise RuntimeError(f"{scenario_file} has fewer than two route waypoints")
    (x0, y0, z0), (x1, y1, _) = points[0], points[1]
    import math
    return x0, y0, z0, math.degrees(math.atan2(y1 - y0, x1 - x0))


def teleport(args):
    x, y, z, yaw = route_start_pose(args.scenario_file)
    # Drop-in like the mapping flow: the trajectory z sits exactly on the
    # road plane; a vehicle spawned a hair inside it falls through and is
    # destroyed below the kill Z.
    z += 0.3
    world = client().get_world()
    hero = find_hero(world)
    hero.set_autopilot(False)
    hero.set_transform(
        carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
    )
    hero.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    for _ in range(3):
        hero.apply_control(carla.VehicleControl(brake=1.0))
        try:
            world.wait_for_tick(2.0)
        except RuntimeError:
            pass
    emit("route_start", x=x, y=y, z=z, yaw_deg=round(yaw, 2))


def await_quiescent(args):
    """Wait until a plain actor query answers promptly: the server spends
    tens of seconds registering the fresh sensor streams after a teleport,
    and SR launched too early wedges on its first client call."""
    for attempt in range(args.attempts):
        started = time.monotonic()
        try:
            world = client(timeout=15.0).get_world()
            world.get_actors()
            elapsed = time.monotonic() - started
            emit("server_quiescent", attempt=attempt, elapsed_s=round(elapsed, 1))
            return
        except Exception:  # noqa: BLE001 - busy server: back off and retry
            time.sleep(args.period_s)
    raise RuntimeError("CARLA server did not become responsive before SR launch")


def verify_world(args):
    """The TCP healthcheck passes while a crashed-and-restarted server is
    still serving the default empty world; a scenario must not start there."""
    world = client(args.timeout).get_world()
    town = world.get_map().name
    heroes = [a for a in world.get_actors() if a.attributes.get("role_name") == "hero"]
    emit("world_verified", map=town, heroes=len(heroes))
    expected = f"Carla/Maps/{args.town}"
    if town != expected or not heroes:
        raise RuntimeError(
            f"CARLA world is {town} with {len(heroes)} heroes; expected "
            f"{expected} with the bridge-spawned hero. The server likely "
            "crashed on boot and reset -- bounce the stack and retry."
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("destroy-hero")
    teleport_p = sub.add_parser("teleport")
    teleport_p.add_argument("scenario_file")
    quiescent_p = sub.add_parser("await-quiescent")
    quiescent_p.add_argument("--attempts", type=int, default=8)
    quiescent_p.add_argument("--period-s", type=float, default=5.0)
    verify_p = sub.add_parser("verify-world")
    verify_p.add_argument("--town", default="Town05")
    verify_p.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    {"destroy-hero": destroy_hero, "teleport": teleport,
     "await-quiescent": await_quiescent, "verify-world": verify_world}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
