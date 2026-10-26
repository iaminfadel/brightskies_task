#!/usr/bin/env python3
"""Interface test for localization_core: node contract without CARLA (#19).

Runs on hosted runners alongside the gtest suite. Launches the real node
(and robot_state_publisher for the sensor TF tree) and asserts the node
interface against the subscribed map contract (ADR 0003, #34):

  1. The estimator takes the map by subscription: a latched
     /localization/map_cloud published *after* the node starts (either-order
     contract) builds the matcher -- observable as /localization/status
     leaving "waiting_for_map". With no sensor input the node then reports
     FAILED with reason "initializing" (initialization requires scans + GNSS
     + IMU; the deeper state machine is covered by test_health.cpp).
  2. A NavSatFix near the configured anchor produces /localization/gnss_pose
     in the map frame with a finite pose and positive covariance (GNSS->ENU
     path against the map.anchor_* parameters; this needs the base_link ->
     hero/gnss extrinsic, which the carla_ros_bridge publishes in the real
     stack and this test provides as static TF).
  3. The fused EKF pose is relayed on /localization/pose as
     PoseWithCovarianceStamped -- the map_server's windowed-serving input.
  4. At runtime the node never subscribes to privileged (ground-truth)
     topics -- the live counterpart of scripts/isolation_audit.py.
"""
import struct
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from brightdrive_interfaces.msg import LocalizationStatus

# Keep in sync with scripts/isolation_audit.py.
PRIVILEGED_SUBSCRIPTIONS = (
    "/carla/hero/odometry",
    "/carla/hero/vehicle_status",
    "/carla/hero/objects",
)
# Default topics.* from config/localization.yaml (map_cloud is the #34
# subscription contract).
EXPECTED_SUBSCRIPTIONS = (
    "/carla/hero/gnss",
    "/localization/map_cloud",
    "/odometry/filtered",
)

ANCHOR_LAT_DEG = 52.370216
ANCHOR_LON_DEG = 4.895168
ANCHOR_ALT_M = 3.7


def make_synthetic_map_cloud():
    """Same 200-point grid as the legacy synthetic map bundle, as PointCloud2
    in the layout the map_server publishes (xyz float32, 12-byte step)."""
    points = [((i % 10) * 0.5, (i // 10) * 0.5, (i % 3) * 0.1) for i in range(200)]
    msg = PointCloud2()
    msg.header = Header(frame_id="map")
    msg.height = 1
    msg.width = len(points)
    for name, offset in (("x", 0), ("y", 4), ("z", 8)):
        field = PointField()
        field.name = name
        field.offset = offset
        field.datatype = PointField.FLOAT32
        field.count = 1
        msg.fields.append(field)
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<3f", x, y, z) for x, y, z in points)
    return msg


def generate_test_description():
    node = launch_ros.actions.Node(
        package="brightdrive_localization",
        executable="localization_core",
        name="localization_core",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "map.anchor_lat_deg": ANCHOR_LAT_DEG,
                "map.anchor_lon_deg": ANCHOR_LON_DEG,
                "map.anchor_alt_m": ANCHOR_ALT_M,
                "map.id": "interface-test-map",
            }
        ],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


# Mirrors the real frame tree: URDF gives base_link -> hero (bringup/urdf),
# the carla_ros_bridge hangs the sensor frames under hero. Any known offset
# works for the contract assertions; these are plausible sensor positions.
STATIC_TF_TREE = (
    ("base_link", "hero", (1.394, 0.0, 0.30)),
    ("hero", "hero/gnss", (0.0, 0.0, 1.80)),
    ("hero", "hero/imu", (0.0, 0.0, 1.60)),
    ("hero", "hero/lidar", (0.0, 0.0, 2.10)),
)


def broadcast_frame_tree(node):
    broadcaster = StaticTransformBroadcaster(node)
    stamps = []
    for parent, child, xyz in STATIC_TF_TREE:
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        stamps.append(transform)
    broadcaster.sendTransform(stamps)


@pytest.mark.launch_testing
class TestLocalizationInterface(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("interface_test")
        # The node's tf_buffer latches /tf_static (transient local), so the
        # late-joining localization_core still receives the frame tree.
        broadcast_frame_tree(cls.node)
        # The map by subscription (#34): the cloud is latched AFTER the node
        # launched, exercising the either-order startup of ADR 0003.
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.map_pub = cls.node.create_publisher(PointCloud2, "/localization/map_cloud", latched)
        cls.map_pub.publish(make_synthetic_map_cloud())

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

    def test_001_status_stream_after_subscribed_map(self):
        states = []
        self.node.create_subscription(
            LocalizationStatus,
            "/localization/status",
            lambda msg: states.append((msg.state, msg.reason)),
            10,
        )
        # Matcher built from the subscribed cloud: the status leaves
        # "waiting_for_map" (proves the subscription contract end to end --
        # the estimator holds FAILED/waiting_for_map until /localization/
        # map_cloud delivers, then initialization can start).
        self.assertTrue(
            self._spin_until(lambda: any(r == "initializing" for _, r in states)),
            f"matcher never built from subscribed cloud; reasons seen: "
            f"{sorted({r for _, r in states})}",
        )
        # ~10 Hz contract (issue #9): collect a healthy sample window.
        self._spin_until(lambda: len(states) >= 20, timeout=20.0)
        post_map = states[-10:]
        self.assertGreaterEqual(len(post_map), 10, f"/localization/status too slow: {states}")
        # No scans/GNSS/IMU: initialization can never start, so the node
        # must hold FAILED with reason "initializing" (not crash, not flip
        # to OK, not stop publishing).
        self.assertTrue(
            all(state == LocalizationStatus.FAILED for state, _ in post_map),
            f"non-FAILED states without inputs: {set(post_map)}",
        )
        self.assertTrue(
            all(reason == "initializing" for _, reason in post_map),
            f"unexpected reasons after map build: {set(post_map)}",
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
        self.assertLess(abs(pose.pose.pose.position.x), 5.0, msg="east offset too large")
        self.assertLess(abs(pose.pose.pose.position.y), 5.0, msg="north offset too large")
        self.assertTrue(
            any(c > 0.0 for c in pose.pose.covariance),
            "gnss_pose covariance not populated",
        )

    def test_003_ekf_pose_relayed_to_localization_pose(self):
        received = []
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose",
            received.append,
            10,
        )
        pub = self.node.create_publisher(Odometry, "/odometry/filtered", 10)
        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = 3.25
        msg.pose.pose.position.y = -1.5
        msg.pose.pose.position.z = 0.4
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.5
        msg.pose.covariance[7] = 0.5
        pub.publish(msg)
        self.assertTrue(
            self._spin_until(lambda: bool(received)),
            "no /localization/pose relay after publishing /odometry/filtered",
        )
        relay = received[-1]
        self.assertEqual(relay.header.frame_id, "map")
        self.assertAlmostEqual(relay.pose.pose.position.x, 3.25, places=5)
        self.assertAlmostEqual(relay.pose.pose.position.y, -1.5, places=5)
        self.assertAlmostEqual(relay.pose.covariance[0], 0.5, places=5)
        self.assertAlmostEqual(relay.pose.covariance[7], 0.5, places=5)

    def test_004_runtime_isolation(self):
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
