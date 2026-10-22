#!/usr/bin/env python3
"""Save one ROS 2 PointCloud2 LiDAR message as a PCD for #18 evidence."""
import argparse
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class ScanCapture(Node):
    def __init__(self, topic: str, path: Path) -> None:
        super().__init__("validation_scan_capture")
        self._path = path
        self._done = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, topic, self._callback, qos)

    def _callback(self, message: PointCloud2) -> None:
        if self._done:
            return
        fields = {field.name: field for field in message.fields}
        missing = {"x", "y", "z"} - fields.keys()
        if missing:
            raise RuntimeError(f"cloud missing fields: {sorted(missing)}")
        count = message.width * message.height
        point_step = message.point_step
        with self._path.open("w", encoding="ascii") as handle:
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
                offset = index * point_step
                x, y, z = struct.unpack_from("fff", message.data, offset)
                intensity = 0.0
                if "intensity" in fields:
                    intensity = struct.unpack_from(
                        "f", message.data, offset + fields["intensity"].offset
                    )[0]
                handle.write(f"{x:.6f} {y:.6f} {z:.6f} {intensity:.6f}\n")
        self._done = True
        print(f"saved {count} points to {self._path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/carla/hero/lidar")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = ScanCapture(args.topic, args.out)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() > deadline:
                print("timed out waiting for a LiDAR scan", file=sys.stderr)
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
