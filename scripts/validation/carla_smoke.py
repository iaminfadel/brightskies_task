#!/usr/bin/env python3
"""CARLA 0.9.15 Python API smoke: save one RGB frame and one LiDAR scan."""
import argparse
import json
import struct
import sys
import time
from pathlib import Path

import carla


def write_pcd(path: Path, scan) -> int:
    raw = scan.raw_data
    point_step = 16
    count = len(raw) // point_step
    with path.open("w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS x y z intensity\n")
        handle.write("SIZE 4 4 4 4\n")
        handle.write("TYPE F F F F\n")
        handle.write("COUNT 1 1 1 1\n")
        handle.write(f"WIDTH {count}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {count}\n")
        handle.write("DATA ascii\n")
        for index in range(count):
            x, y, z, intensity = struct.unpack_from("ffff", raw, index * point_step)
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.6f}\n")
    return count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--town", default="Town10HD")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    started = time.time()
    world = client.get_world()
    if args.town and args.town not in world.get_map().name:
        world = client.load_world(args.town)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    client_version = client.get_client_version()
    server_version = client.get_server_version()

    blueprints = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("CARLA map has no spawn points")

    vehicle = None
    spawn_index = -1
    for index, transform in enumerate(spawn_points):
        candidate = world.try_spawn_actor(
            blueprints.find("vehicle.lincoln.mkz_2017"), transform
        )
        if candidate is not None:
            vehicle = candidate
            spawn_index = index
            break
    if vehicle is None:
        raise RuntimeError("could not spawn a vehicle at any spawn point")

    camera_bp = blueprints.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "1280")
    camera_bp.set_attribute("image_size_y", "720")
    camera_bp.set_attribute("fov", "90")
    camera_bp.set_attribute("sensor_tick", "0.1")

    lidar_bp = blueprints.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("range", "50")
    lidar_bp.set_attribute("channels", "16")
    lidar_bp.set_attribute("points_per_second", "288000")
    lidar_bp.set_attribute("rotation_frequency", "10")
    lidar_bp.set_attribute("upper_fov", "15")
    lidar_bp.set_attribute("lower_fov", "-15")
    lidar_bp.set_attribute("noise_stddev", "0")

    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(carla.Location(x=2.0, z=2.0)),
        attach_to=vehicle,
    )
    lidar = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(z=2.4)),
        attach_to=vehicle,
    )

    frames = {}
    camera.listen(lambda image: frames.setdefault("camera", image))
    lidar.listen(lambda scan: frames.setdefault("lidar", scan))

    deadline = time.time() + args.timeout
    while time.time() < deadline and not ("camera" in frames and "lidar" in frames):
        world.tick()
        time.sleep(0.01)

    if "camera" not in frames:
        raise RuntimeError("no RGB frame received before timeout")
    if "lidar" not in frames:
        raise RuntimeError("no LiDAR scan received before timeout")

    image_path = args.out / "rgb_frame.png"
    pcd_path = args.out / "lidar_scan.pcd"
    frames["camera"].save_to_disk(str(image_path))
    point_count = write_pcd(pcd_path, frames["lidar"])

    summary = {
        "client_version": client_version,
        "server_version": server_version,
        "map": world.get_map().name,
        "spawn_index": spawn_index,
        "rgb_frame": str(image_path),
        "lidar_scan": str(pcd_path),
        "lidar_points": point_count,
        "elapsed_s": round(time.time() - started, 3),
    }
    (args.out / "smoke_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))

    lidar.stop()
    lidar.destroy()
    camera.stop()
    camera.destroy()
    vehicle.destroy()
    return 0


if __name__ == "__main__":
    sys.exit(main())
