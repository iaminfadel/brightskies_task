#!/usr/bin/env python3
"""Wait until a ROS topic (and optionally a publisher) is visible."""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--publisher", action="store_true")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("wait_graph_probe")
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            if args.publisher:
                seen = node.count_publishers(args.topic) > 0
            else:
                seen = any(
                    name == args.topic
                    for name, _ in node.get_topic_names_and_types()
                )
            if seen:
                return 0
            time.sleep(0.5)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
