#!/usr/bin/env python3
"""Interface test for brightdrive_map_server (ADR 0003, #33).

Launches the real node three times against one synthetic tiled bundle --
the map_index.yaml + tiles/ contract emitted by scripts/build_map.py (#32):

  1. full mode publishes the whole map.pcd once, latched: a late-joining
     transient-local subscriber still receives it.
  2. windowed mode falls back to the full map while the pose source is
     absent, serves the R_load window once /localization/pose flows, and
     falls back to full again when the pose goes stale.
  3. auto mode publishes the full map first (the startup-loop break), then
     switches to the windowed set seeded by initial_pose, and follows the
     topic pose once it arrives.

Every tile carries a distinct point count, so the served width identifies
exactly which set is published.
"""
import json
import math
import os
import struct
import tempfile
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

TILE_SIZE = 10.0
LIDAR_RANGE_M = 5.0
MARGIN_M = 10.0           # R_load = 15 m
HYSTERESIS_M = 15.0       # R_evict = 30 m
POSE_STALE_S = 1.0



def tile_point_count(ix, iy):
    return 100 + 10 * ix + iy


def served_window(loaded_tiles, x, y):
    """The serving set the node must produce: tiles within R_load, plus tiles
    already loaded that are still within R_evict (hysteresis)."""
    r_load = LIDAR_RANGE_M + MARGIN_M
    r_evict = r_load + HYSTERESIS_M
    result = set()
    for ix in range(3):
        for iy in range(3):
            distance = math.hypot((ix + 0.5) * TILE_SIZE - x, (iy + 0.5) * TILE_SIZE - y)
            if distance <= r_load or ((ix, iy) in loaded_tiles and distance <= r_evict):
                result.add((ix, iy))
    return result


# Fresh-load window about (5, 5): tile centres within R_load = 15 m.
SEED_WINDOW_TILES = {(0, 0), (1, 0), (0, 1), (1, 1)}
# Fresh-load window about the auto seed pose (25, 5).
AUTO_SEED_TILES = served_window(set(), 25.0, 5.0)


def full_point_count():
    return sum(tile_point_count(ix, iy) for ix in range(3) for iy in range(3))


def window_point_count(tiles):
    return sum(tile_point_count(ix, iy) for ix, iy in tiles)


def write_pcd_xyz(path, count):
    payload = b"".join(
        struct.pack("<3f", float(i % 100), float(i % 37), 0.0) for i in range(count)
    )
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as handle:
        handle.write(header + payload)


def write_bundle(directory):
    """Full map.pcd + tiles/ + map_index.yaml, the #32 bundle contract."""
    tiles_dir = os.path.join(directory, "tiles")
    os.makedirs(tiles_dir)
    files = []
    for iy in range(3):
        for ix in range(3):
            count = tile_point_count(ix, iy)
            write_pcd_xyz(os.path.join(tiles_dir, f"tile_{ix}_{iy}.pcd"), count)
            files.append(
                {
                    "path": f"tiles/tile_{ix}_{iy}.pcd",
                    "ix": ix,
                    "iy": iy,
                    "point_count": count,
                }
            )
    write_pcd_xyz(os.path.join(directory, "map.pcd"), full_point_count())
    index = {
        "schema_version": 1,
        "map_id": "map-server-interface-test",
        "map_frame": "map",
        "tile_size_m": TILE_SIZE,
        "origin": {"x_m": 0.0, "y_m": 0.0},
        "z_range_m": [0.0, 0.0],
        "tile_count": len(files),
        "point_count": full_point_count(),
        "files": files,
    }
    # JSON text: exactly what build_map.py writes (valid YAML 1.2).
    with open(os.path.join(directory, "map_index.yaml"), "w", encoding="ascii") as handle:
        handle.write(json.dumps(index, indent=2))


def map_server_node(name, mode, cloud_topic, pose_topic, bundle_dir, seed=None):
    parameters = {
        "use_sim_time": False,
        "serving.mode": mode,
        "serving.map_dir": bundle_dir,
        "serving.lidar_range_m": LIDAR_RANGE_M,
        "serving.margin_m": MARGIN_M,
        "serving.hysteresis_m": HYSTERESIS_M,
        "serving.pose_stale_s": POSE_STALE_S,
        "serving.pose_topic": pose_topic,
    }
    if seed is not None:
        parameters["serving.initial_pose_x"] = seed[0]
        parameters["serving.initial_pose_y"] = seed[1]
    return launch_ros.actions.Node(
        package="brightdrive_map_server",
        executable="map_server",
        name=name,
        output="screen",
        parameters=[parameters],
        remappings=[("/localization/map_cloud", cloud_topic)],
    )


def generate_test_description():
    bundle_dir = tempfile.mkdtemp(prefix="bd_map_server_bundle_")
    write_bundle(bundle_dir)
    return launch.LaunchDescription(
        [
            map_server_node("ms_full", "full", "/ms_test/full_cloud", "/ms_test/unused",
                            bundle_dir),
            map_server_node("ms_windowed", "windowed", "/ms_test/windowed_cloud",
                            "/ms_test/pose", bundle_dir),
            map_server_node("ms_auto", "auto", "/ms_test/auto_cloud", "/ms_test/pose_auto",
                            bundle_dir, seed=(25.0, 5.0)),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def collect_widths(node, topic):
    """Collect (timestamp, width) from a transient-local subscription."""
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    widths = []
    node.create_subscription(
        PointCloud2, topic, lambda msg: widths.append((time.monotonic(), msg.width)), qos
    )
    return widths


class TestMapServerInterface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("map_server_interface_test")
        cls.full_count = full_point_count()
        cls.pose_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, "/ms_test/pose", 10
        )
        # The auto node's full-first publish happens seconds before test_003
        # runs, and depth-1 latching keeps only the latest sample -- so the
        # subscription must exist from the very start to observe the order.
        cls.auto_widths = collect_widths(cls.node, "/ms_test/auto_cloud")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _subscribe_widths(self, topic):
        return collect_widths(self.node, topic)

    def _spin_until(self, done, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if done():
                return True
        return False

    def _publish_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self.pose_pub.publish(msg)

    def _publish_pose_until(self, x, y, done, timeout=5.0):
        def drive():
            while rclpy.ok() and not done():
                self._publish_pose(x, y)
                time.sleep(0.05)

        worker = threading.Thread(target=drive, daemon=True)
        worker.start()
        return self._spin_until(done, timeout=timeout)

    def test_001_full_mode_is_latched(self):
        widths = self._subscribe_widths("/ms_test/full_cloud")
        self.assertTrue(
            self._spin_until(lambda: any(w == self.full_count for _, w in widths)),
            f"full map never published: {widths}",
        )
        # Late joiner: a fresh subscription still receives the latched cloud.
        late = self._subscribe_widths("/ms_test/full_cloud")
        self.assertTrue(
            self._spin_until(lambda: any(w == self.full_count for _, w in late), timeout=5.0),
            f"transient-local latching broken for a late joiner: {late}",
        )

    def test_002_windowed_fallback_and_window(self):
        widths = self._subscribe_widths("/ms_test/windowed_cloud")
        # Pose source absent: the full map is the fallback.
        self.assertTrue(
            self._spin_until(lambda: any(w == self.full_count for _, w in widths)),
            f"no full-map fallback without a pose: {widths}",
        )
        # Pose flows at (5, 5): the window set appears.
        expected = window_point_count(SEED_WINDOW_TILES)
        self.assertTrue(
            self._publish_pose_until(
                5.0, 5.0, lambda: any(w == expected for _, w in widths)
            ),
            f"window never served {expected} pts: {widths}",
        )
        # Pose goes stale: back to the full map.
        seen_before_stale = len(widths)
        self.assertTrue(
            self._spin_until(
                lambda: any(w == self.full_count for _, w in widths[seen_before_stale:]),
                timeout=10.0,
            ),
            f"no full-map fallback after the pose went stale: {widths}",
        )

    def test_003_auto_full_first_then_seed_window_then_topic(self):
        widths = self.auto_widths  # observed from before the nodes started
        pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/ms_test/pose_auto", 10
        )
        # The first publish is the full map (startup-loop break), then the
        # seeded window at (25, 5).
        self.assertTrue(
            self._spin_until(
                lambda: any(w == window_point_count(AUTO_SEED_TILES) for _, w in widths)
            ),
            f"auto never reached the seeded window: {widths}",
        )
        self.assertTrue(
            any(w == self.full_count for _, w in widths),
            f"auto did not publish the full map first: {widths}",
        )
        first_full = next(
            i for i, (_, w) in enumerate(widths) if w == self.full_count
        )
        first_window = next(
            i for i, (_, w) in enumerate(widths)
            if w == window_point_count(AUTO_SEED_TILES)
        )
        self.assertLess(first_full, first_window, "window served before the full-map fallback")
        def publish_pose(x, y):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            pose_pub.publish(msg)

        def publish_until(x, y, done):
            def drive():
                while rclpy.ok() and not done():
                    publish_pose(x, y)
                    time.sleep(0.05)

            worker = threading.Thread(target=drive, daemon=True)
            worker.start()
            return self._spin_until(done, timeout=5.0)

        # Topic pose at (5, 5) takes over from the seed; tiles of the seed
        # window that are still within R_evict survive the swap.
        expected = window_point_count(served_window(AUTO_SEED_TILES, 5.0, 5.0))
        self.assertTrue(
            publish_until(5.0, 5.0, lambda: any(w == expected for _, w in widths)),
            f"topic pose never took over the window: {widths}",
        )
