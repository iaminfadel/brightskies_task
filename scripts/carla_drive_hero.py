#!/usr/bin/env python3
"""Drive the spawned hero with the CARLA Traffic Manager (labelled harness).

Used only to produce mapping bags and as the open-loop control fallback for the
scenario harness. The estimator never sees this process.
"""
import argparse
import math
import time

import carla


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero")
    parser.add_argument("--speed-kmh", type=float, default=30.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    hero = None
    deadline = time.time() + 60.0
    while hero is None and time.time() < deadline:
        try:
            world = client.get_world()
            world.wait_for_tick(2.0)
            for actor in world.get_actors():
                if actor.attributes.get("role_name") == args.role:
                    hero = actor
                    break
        except RuntimeError:
            pass
        if hero is None:
            time.sleep(1.0)
    if hero is None:
        raise SystemExit(f"no actor with role_name={args.role}")

    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    hero.set_autopilot(True, 8000)
    speed_mps = args.speed_kmh / 3.6
    try:
        tm.set_desired_speed(hero, speed_mps)
    except AttributeError:
        tm.vehicle_percentage_speed_difference(hero, 0)

    deadline = time.time() + args.seconds if args.seconds > 0 else None
    last = time.time()
    while deadline is None or time.time() < deadline:
        pose = hero.get_transform()
        if time.time() - last >= 5.0:
            print(f"hero x={pose.location.x:.1f} y={pose.location.y:.1f} "
                  f"yaw={pose.rotation.yaw:.1f} v={hero.get_velocity().length() * 3.6:.1f} km/h",
                  flush=True)
            last = time.time()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
