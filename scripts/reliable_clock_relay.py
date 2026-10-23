#!/usr/bin/env python3
"""Re-publish /clock with reliable QoS so node time sources can subscribe.

rosbag2 publishers expose /clock as best effort; rclcpp time sources subscribe
reliably, so replayed sim time would otherwise be ignored.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockRelay(Node):
    def __init__(self) -> None:
        super().__init__("clock_relay")
        source_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        relay_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._last = None
        self._pub = self.create_publisher(Clock, "/clock", relay_qos)
        self.create_subscription(Clock, "/clock", self._relay, source_qos)

    def _relay(self, msg: Clock) -> None:
        stamp = (msg.clock.sec, msg.clock.nanosec)
        if stamp == self._last:
            return
        self._last = stamp
        self._pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = ClockRelay()
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
