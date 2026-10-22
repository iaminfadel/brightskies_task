#!/usr/bin/env python3
"""Pure-pursuit route driver for empty-town mapping runs.

The mapping bag records sensors only; this driver is the labelled open-loop
harness that keeps the ego on the configured Town05 loop. It talks to CARLA
directly so it does not need the ROS control path.
"""

import argparse
import math
import sys
import time

import carla

sys.path.insert(0, "/opt/carla/PythonAPI/carla")
from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402

DEFAULT_SPAWNS = "0,203,144,270,208,0"


def wait_hero(world, role, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == role:
                return actor
        time.sleep(0.5)
    raise RuntimeError(f"no hero vehicle with role {role}")


def build_route(grp, spawns, indices):
    route = []
    for a, b in zip(indices, indices[1:]):
        leg = grp.trace_route(spawns[a].location, spawns[b].location)
        for waypoint, _ in leg:
            p = waypoint.transform.location
            if route and route[-1].distance(p) < 1e-3:
                continue
            route.append(p)
    return route


def route_headings(route):
    headings = []
    for i in range(len(route)):
        a = route[max(0, i - 1)]
        b = route[min(len(route) - 1, i + 1)]
        headings.append(math.atan2(b.y - a.y, b.x - a.x))
    return headings


def turn_ahead(headings, idx, count):
    if idx + count >= len(headings):
        count = len(headings) - idx - 1
    if count <= 1:
        return 0.0
    delta = headings[idx + count] - headings[idx]
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--role", default="hero")
    parser.add_argument("--spawns", default=DEFAULT_SPAWNS)
    parser.add_argument("--speed-kmh", type=float, default=30.0)
    parser.add_argument("--lookahead-m", type=float, default=8.0)
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--start-delay-s", type=float, default=2.0)
    parser.add_argument("--stuck-grace-s", type=float, default=8.0)
    parser.add_argument("--no-teleport", action="store_true")
    parser.add_argument("--ignore-lights", action="store_true")
    parser.add_argument("--log-period-s", type=float, default=5.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    hero = wait_hero(world, args.role, 60.0)
    wmap = world.get_map()
    spawns = wmap.get_spawn_points()
    indices = [int(x) for x in args.spawns.split(",")]
    grp = GlobalRoutePlanner(wmap, 1.0)
    route = build_route(grp, spawns, indices)
    headings = route_headings(route)
    length = sum(route[i].distance(route[i + 1]) for i in range(len(route) - 1))
    print(
        f"ROUTE points={len(route)} length={length:.1f}m spawns={indices} "
        f"speed={args.speed_kmh:.0f}km/h",
        flush=True,
    )

    hero.set_autopilot(False)
    if not args.no_teleport:
        hero.set_transform(spawns[indices[0]])
        hero.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        hero.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
    for _ in range(10):
        hero.apply_control(carla.VehicleControl(brake=1.0))
        world.wait_for_tick(2.0)

    if args.start_delay_s > 0:
        deadline = time.time() + args.start_delay_s
        while time.time() < deadline:
            hero.apply_control(carla.VehicleControl(brake=1.0))
            world.wait_for_tick(2.0)
    print("DRIVE_START", flush=True)

    target_speed = args.speed_kmh / 3.6
    idx = 0
    started = time.time()
    last_log = 0.0
    stuck_since = None
    reversing_until = 0.0
    reverse_attempts = 0
    while True:
        world.wait_for_tick(2.0)
        if time.time() - started >= args.seconds:
            print("DRIVE_TIMEOUT", flush=True)
            break

        tf = hero.get_transform()
        loc = tf.location
        yaw = math.radians(tf.rotation.yaw)
        velocity = hero.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        best = idx
        best_d = loc.distance(route[idx])
        for j in range(idx, min(idx + 250, len(route))):
            d = loc.distance(route[j])
            if d < best_d:
                best_d = d
                best = j
        idx = best

        lookahead = max(4.0, min(args.lookahead_m, 0.9 * speed + 3.0))
        target = idx
        accum = 0.0
        while target + 1 < len(route) and accum < lookahead:
            accum += route[target].distance(route[target + 1])
            target += 1
        if target >= len(route) - 1 and loc.distance(route[-1]) < 6.0:
            print("ROUTE_COMPLETE", flush=True)
            break

        dx = route[target].x - loc.x
        dy = route[target].y - loc.y
        fx = math.cos(yaw) * dx + math.sin(yaw) * dy
        fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
        steer = math.atan2(fy, fx) / math.radians(50.0)
        steer = max(-1.0, min(1.0, 1.1 * steer))

        desired = target_speed
        turn = turn_ahead(headings, idx, 15)
        if turn > math.radians(45.0):
            desired = min(desired, 4.0)
        elif turn > math.radians(20.0):
            desired = min(desired, 6.5)

        light = None if args.ignore_lights else hero.get_traffic_light()
        if light is not None:
            state = hero.get_traffic_light_state()
            lx = light.get_location().x - loc.x
            ly = light.get_location().y - loc.y
            ahead = math.cos(yaw) * lx + math.sin(yaw) * ly
            dist = math.sqrt(lx * lx + ly * ly)
            if state == carla.TrafficLightState.Red and ahead > 0 and dist < 30.0:
                desired = 0.0
            elif state == carla.TrafficLightState.Yellow and ahead > 0 and dist < 12.0:
                desired = 0.0

        now = time.time()
        if now < reversing_until:
            hero.apply_control(
                carla.VehicleControl(throttle=0.4, steer=-steer, brake=0.0, reverse=True)
            )
        else:
            if speed < 0.25 and desired > 1.0 and now - started > args.stuck_grace_s:
                if stuck_since is None:
                    stuck_since = now
                elif now - stuck_since > 4.0:
                    reverse_attempts += 1
                    if reverse_attempts > 3:
                        print("DRIVE_STUCK", flush=True)
                        hero.apply_control(carla.VehicleControl(brake=1.0))
                        return 2
                    print(f"RECOVER reverse={reverse_attempts}", flush=True)
                    reversing_until = now + 2.5
                    stuck_since = None
                    hero.apply_control(
                        carla.VehicleControl(
                            throttle=0.4, steer=-steer, brake=0.0, reverse=True
                        )
                    )
            else:
                stuck_since = None
                err = desired - speed
                if err >= 0.0:
                    throttle = min(0.75, 0.3 + 0.25 * err)
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = min(0.6, -0.25 * err)
                hero.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
                )

        if now - last_log >= args.log_period_s:
            last_log = now
            pct = 100.0 * min(1.0, idx / max(1, len(route) - 1))
            print(
                f"PROGRESS pct={pct:.1f} idx={idx}/{len(route)} "
                f"speed={speed*3.6:.1f}km/h desired={desired*3.6:.0f} "
                f"x={loc.x:.1f} y={loc.y:.1f} yaw={tf.rotation.yaw:.1f}",
                flush=True,
            )

    hero.apply_control(carla.VehicleControl(brake=1.0))
    print("DRIVE_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
