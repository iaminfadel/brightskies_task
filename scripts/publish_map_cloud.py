#!/usr/bin/env python3
"""Publish a saved map PCD as a latched PointCloud2 for visualization only."""
import argparse
import pathlib
import struct

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd(path: pathlib.Path) -> np.ndarray:
    raw = path.read_bytes()
    marker = b"DATA binary\n"
    end = raw.index(marker) + len(marker)
    header = raw[:end].decode("ascii", errors="replace")
    fields = None
    points = None
    for line in header.splitlines():
        key, _, value = line.partition(" ")
        if key == "FIELDS":
            fields = value.split()
        elif key == "POINTS":
            points = int(value)
    if fields != ["x", "y", "z"]:
        raise ValueError(f"{path}: expected x y z fields, got {fields}")
    count = points if points is not None else (len(raw) - end) // 12
    cloud = np.frombuffer(raw, dtype="<f4", count=count * 3, offset=end)
    return np.asarray(cloud, dtype=np.float32).reshape(-1, 3)


class MapCloudPublisher(Node):
    def __init__(self, cloud: np.ndarray, topic: str, frame: str, period: float):
        super().__init__("map_cloud_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cloud = cloud
        self._topic = topic
        self._frame = frame
        self._pub = self.create_publisher(PointCloud2, topic, qos)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f"publishing {len(cloud)} points on {topic} every {period:.1f}s"
        )

    def _publish(self) -> None:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.height = 1
        msg.width = len(self._cloud)
        msg.fields = [
            PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            for name, offset in (("x", 0), ("y", 4), ("z", 8))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = self._cloud.tobytes()
        self._pub.publish(msg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-dir", default="/ws/maps/current/town05-v2")
    parser.add_argument("--topic", default="/localization/map_cloud")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--period", type=float, default=2.0)
    args = parser.parse_args()

    path = pathlib.Path(args.map_dir) / "map.pcd"
    cloud = read_pcd(path)
    rclpy.init()
    node = MapCloudPublisher(cloud, args.topic, args.frame, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
