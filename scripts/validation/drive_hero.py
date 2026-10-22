#!/usr/bin/env python3
"""Enable CARLA autopilot on the bridge-spawned `hero` actor."""
import argparse
import time

import carla


def find_hero(world, role_name):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role-name", default="hero")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    while time.monotonic() < deadline:
        world = client.get_world()
        actor = find_hero(world, args.role_name)
        if actor is not None:
            actor.set_autopilot(True)
            print(f"autopilot enabled for {args.role_name} ({actor.id})")
            return 0
        try:
            world.wait_for_tick(1.0)
        except RuntimeError:
            time.sleep(0.5)
    raise SystemExit(f"no vehicle with role_name={args.role_name}")


if __name__ == "__main__":
    raise SystemExit(main())
