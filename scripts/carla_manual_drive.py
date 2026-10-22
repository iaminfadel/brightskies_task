#!/usr/bin/env python3
"""Open-loop throttle/steer harness for short mapping bags.

Connects to CARLA, waits for the hero to appear in a snapshot, optionally
teleports it to a map spawn point, then applies a constant throttle (and
optional steer) for a fixed simulation-time interval.
"""
import argparse
import time

import carla


def find_hero(client, role, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            world = client.get_world()
            world.wait_for_tick(2.0)
            for actor in world.get_actors():
                if actor.attributes.get("role_name") == role:
                    return world, actor
        except RuntimeError:
            pass
        time.sleep(1.0)
    return None, None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero")
    parser.add_argument("--spawn-index", type=int, default=0)
    parser.add_argument("--teleport", action="store_true")
    parser.add_argument("--throttle", type=float, default=0.35)
    parser.add_argument("--steer", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world, hero = find_hero(client, args.role, 90.0)
    if hero is None:
        raise SystemExit("hero not found")

    hero.set_autopilot(False)
    if args.teleport:
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise SystemExit("map has no spawn points")
        spawn = spawn_points[args.spawn_index % len(spawn_points)]
        hero.set_transform(spawn)
        world.wait_for_tick(5.0)
    start = time.time()
    last = start
    while time.time() - start < args.seconds:
        hero.apply_control(carla.VehicleControl(throttle=args.throttle, steer=args.steer))
        time.sleep(0.05)
        if time.time() - last >= 3.0:
            pose = hero.get_transform()
            print(f"x={pose.location.x:.1f} y={pose.location.y:.1f} "
                  f"yaw={pose.rotation.yaw:.1f} v={hero.get_velocity().length() * 3.6:.1f}",
                  flush=True)
            last = time.time()
    hero.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
    print("done", flush=True)


if __name__ == "__main__":
    main()
