#!/usr/bin/env python3
"""Save one ROS 2 Image message as a PNG for #18 evidence."""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


class FrameCapture(Node):
    def __init__(self, topic: str, path: Path) -> None:
        super().__init__("validation_frame_capture")
        self._path = path
        self._done = False
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, topic, self._callback, qos)

    def _callback(self, message: Image) -> None:
        if self._done:
            return
        if message.encoding not in CHANNELS:
            raise RuntimeError(f"unsupported encoding: {message.encoding}")
        count = CHANNELS[message.encoding]
        row_bytes = message.width * count
        raw = np.frombuffer(message.data, dtype=np.uint8)
        if message.step != row_bytes:
            raw = raw.reshape(message.height, message.step)[:, :row_bytes]
        frame = raw.reshape(message.height, message.width, count)
        if message.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        if count == 4:
            frame = frame[:, :, :3]
        cv2.imwrite(str(self._path), frame)
        self._done = True
        print(
            f"saved {message.width}x{message.height} "
            f"{message.encoding} to {self._path}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/carla/hero/camera_front/image")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = FrameCapture(args.topic, args.out)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() > deadline:
                print("timed out waiting for an image frame", file=sys.stderr)
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
