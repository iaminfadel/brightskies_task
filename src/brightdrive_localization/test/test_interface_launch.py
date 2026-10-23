#!/usr/bin/env python3
"""Interface test for localization_core: node contract without CARLA (#19).

Runs on hosted runners alongside the gtest suite. Launches the real node
(and robot_state_publisher for the sensor TF tree) against a synthetic map
bundle (binary PCD + map.yaml, same contract as test_map_loader.cpp) and
asserts the node interface:

  1. /localization/status streams at ~10 Hz; with no sensor input the node
     reports FAILED with reason "initializing" (initialization requires
     scans + GNSS + IMU; the deeper state machine is covered by
     test_health.cpp).
  2. A NavSatFix near the anchor produces /localization/gnss_pose in the map
     frame with a finite pose and positive covariance (GNSS->ENU path; this
     needs the base_link -> hero/gnss extrinsic from the URDF).
  3. At runtime the node never subscribes to privileged (ground-truth)
     topics -- the live counterpart of scripts/isolation_audit.py.
"""
import atexit
import hashlib
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
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from brightdrive_interfaces.msg import LocalizationStatus

# Keep in sync with scripts/isolation_audit.py.
PRIVILEGED_SUBSCRIPTIONS = (
    "/carla/hero/odometry",
    "/carla/hero/vehicle_status",
    "/carla/hero/objects",
)
# Default topics.gnss / topics.filtered from config/localization.yaml.
EXPECTED_SUBSCRIPTIONS = (
    "/carla/hero/gnss",
    "/odometry/filtered",
)

ANCHOR_LAT_DEG = 52.370216
ANCHOR_LON_DEG = 4.895168
ANCHOR_ALT_M = 3.7


def write_synthetic_map(directory):
    """Minimal valid bundle: 200-point grid PCD (binary xyz) + map.yaml."""
    points = [
        ((i % 10) * 0.5, (i // 10) * 0.5, (i % 3) * 0.1) for i in range(200)
    ]
    payload = b"".join(struct.pack("<3f", x, y, z) for x, y, z in points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    pcd_path = os.path.join(directory, "map.pcd")
    with open(pcd_path, "wb") as handle:
        handle.write(header + payload)
    with open(pcd_path, "rb") as handle:
        digest = hashlib.sha256(handle.read()).hexdigest()
    with open(os.path.join(directory, "map.yaml"), "w", encoding="ascii") as handle:
        handle.write(
            f"schema_version: 1\n"
            f"map_id: interface-test-map\n"
            f"map_frame: map\n"
            f"anchor:\n"
            f"  lat_deg: {ANCHOR_LAT_DEG}\n"
            f"  lon_deg: {ANCHOR_LON_DEG}\n"
            f"  alt_m: {ANCHOR_ALT_M}\n"
            f"  stamp: 1.5\n"
            f"T_map_slam: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
            f"pcd_sha256: {digest}\n"
            f"voxel_size_m: 0.25\n"
            f"point_count: {len(points)}\n"
            f"coverage_radius_m: 20.0\n"
            f"source:\n"
            f"  run_id: interface-test\n"
        )
    return pcd_path


def generate_test_description():
    map_dir = tempfile.mkdtemp(prefix="bd_interface_map_")
    atexit.register(lambda: __import__("shutil").rmtree(map_dir, ignore_errors=True))
    write_synthetic_map(map_dir)

    # The extrinsics (base_link -> hero/gnss etc.) come from the URDF via
    # robot_state_publisher; without them publish_gnss_pose bails at the
    # TF lookup and the GNSS path is untestable.
    urdf_path = os.path.join(
        get_package_share_directory("brightdrive_bringup"), "urdf", "brightdrive.urdf"
    )
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )
    node = launch_ros.actions.Node(
        package="brightdrive_localization",
        executable="localization_core",
        name="localization_core",
        output="screen",
        parameters=[{"use_sim_time": False, "map.dir": map_dir}],
    )
    return launch.LaunchDescription(
        [state_publisher, node, launch_testing.actions.ReadyToTest()]
    )


@pytest.mark.launch_testing
class TestLocalizationInterface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("interface_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, done, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if done():
                return True
        return False

    def test_001_status_stream_without_inputs(self):
        states = []
        self.node.create_subscription(
            LocalizationStatus,
            "/localization/status",
            lambda msg: states.append((msg.state, msg.reason)),
            10,
        )
        # ~10 Hz contract (issue #9): collect a healthy sample window.
        self._spin_until(lambda: len(states) >= 10, timeout=20.0)
        self.assertGreaterEqual(
            len(states), 10, f"/localization/status too slow: {len(states)} msgs"
        )
        # No scans/GNSS/IMU: initialization can never start, so the node
        # must hold FAILED with reason "initializing" (not crash, not flip
        # to OK, not stop publishing).
        self.assertTrue(
            all(state == LocalizationStatus.FAILED for state, _ in states),
            f"non-FAILED states without inputs: {set(states)}",
        )
        self.assertTrue(
            all(reason == "initializing" for _, reason in states),
            f"unexpected reasons without inputs: {set(states)}",
        )

    def test_002_gnss_to_gnss_pose(self):
        received = []
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            received.append,
            10,
        )
        pub = self.node.create_publisher(NavSatFix, "/carla/hero/gnss", 10)
        msg = NavSatFix()
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        # ~1.1 m north of the anchor.
        msg.latitude = ANCHOR_LAT_DEG + 1e-5
        msg.longitude = ANCHOR_LON_DEG
        msg.altitude = ANCHOR_ALT_M
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [2.25, 0.0, 0.0, 0.0, 2.25, 0.0, 0.0, 0.0, 9.0]

        def publish_until_seen():
            while rclpy.ok() and not received:
                msg.header.stamp = self.node.get_clock().now().to_msg()
                pub.publish(msg)
                time.sleep(0.1)

        worker = threading.Thread(target=publish_until_seen, daemon=True)
        worker.start()
        self.assertTrue(
            self._spin_until(lambda: bool(received)),
            "no /localization/gnss_pose after publishing NavSatFix",
        )
        pose = received[-1]
        self.assertEqual(pose.header.frame_id, "map")
        self.assertLess(abs(pose.pose.position.x), 5.0, msg="east offset too large")
        self.assertLess(abs(pose.pose.position.y), 5.0, msg="north offset too large")
        self.assertTrue(
            any(c > 0.0 for c in pose.pose.covariance),
            "gnss_pose covariance not populated",
        )

    def test_003_runtime_isolation(self):
        found = self.node.get_subscriber_names_and_types_by_node(
            "localization_core", "/"
        )
        names = sorted(name for name, _ in found)
        for privileged in PRIVILEGED_SUBSCRIPTIONS:
            self.assertNotIn(
                privileged, names, f"privileged subscription live: {privileged}"
            )
        for expected in EXPECTED_SUBSCRIPTIONS:
            self.assertIn(expected, names, f"expected subscription missing: {names}")
