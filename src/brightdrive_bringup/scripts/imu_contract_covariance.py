#!/usr/bin/env python3
"""Relay CARLA IMU with the contract #9 measurement covariance.

CARLA publishes zero covariance for every IMU field. robot_localization treats
a zero covariance as a perfect measurement, so this relay stamps the modelled
noise (orientation 0.5 deg, angular rate 0.01 rad/s) on the diagonal and
republishes on /localization/imu. Values are not filtered or altered.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

ORIENTATION_VAR = math.radians(0.5) ** 2
GYRO_VAR = 0.01 ** 2


class ImuContractCovariance(Node):
    def __init__(self):
        super().__init__("imu_contract_covariance")
        self._pub = self.create_publisher(Imu, "/localization/imu", qos_profile_sensor_data)
        self.create_subscription(Imu, "/carla/hero/imu", self._on_imu, qos_profile_sensor_data)

    def _on_imu(self, msg: Imu) -> None:
        orientation = msg.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if not math.isfinite(norm) or norm < 1e-6:
            # CARLA emits a zero quaternion before the first physics tick;
            # forwarding it would poison the EKF state with NaNs.
            return
        if not all(
            math.isfinite(value)
            for value in (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        ):
            return

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation_covariance = [
            ORIENTATION_VAR, 0.0, 0.0,
            0.0, ORIENTATION_VAR, 0.0,
            0.0, 0.0, ORIENTATION_VAR,
        ]
        out.angular_velocity_covariance = [
            GYRO_VAR, 0.0, 0.0,
            0.0, GYRO_VAR, 0.0,
            0.0, 0.0, GYRO_VAR,
        ]
        out.linear_acceleration_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, 1.0e6,
        ]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ImuContractCovariance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
