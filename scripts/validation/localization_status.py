#!/usr/bin/env python3
"""Placeholder /localization/status publisher for runtime validation only.

The production status message is a #9/#20 contract; this node exists so the
#18 readiness path can be exercised end to end. It publishes a plain String.
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LocalizationStatus(Node):
    def __init__(self, status: str, rate_hz: float) -> None:
        super().__init__("localization_status")
        self._publisher = self.create_publisher(String, "/localization/status", 10)
        self._message = String()
        self._message.data = status
        self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        self._publisher.publish(self._message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status", default="OK")
    parser.add_argument("--rate", type=float, default=10.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = LocalizationStatus(args.status, args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
