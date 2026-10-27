#!/usr/bin/env python3
"""Apply the configured GNSS degradation profile to the raw CARLA GNSS.

Test harness node (#37, #10 §5): subscribes /carla/hero/gnss and republishes
on /localization/gnss with the profile's Gaussian noise, constant east/north
bias, and dropout windows applied. The profile (sigmas, bias, windows, seed)
is read from the same gnss_profiles.yaml the estimator's covariance config
comes from, so the sensor model and the filter cannot drift apart.

The nominal profile still applies its (small) noise: the rig's CARLA GNSS
sensor ships with zero sensor noise, and the profile owns the noise model.
During a dropout window the relay simply stops publishing, so GNSS ages
past gnss_timeout_s and the health state degrades -- the filter is never
told the dropout is coming.
"""
import os

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from ament_index_python.packages import get_package_share_directory


def _load_profile(path, name):
    with open(path, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle)
    profiles = doc["profiles"]
    if name not in profiles:
        raise KeyError(f"GNSS profile {name!r} not in {path}; have {sorted(profiles)}")
    return doc["degrees_per_metre"], profiles[name]


class GnssRelay(Node):
    def __init__(self):
        super().__init__("gnss_relay")
        default_profiles = os.path.join(
            get_package_share_directory("brightdrive_bringup"), "config", "gnss_profiles.yaml"
        )
        profiles_file = os.environ.get("GNSS_PROFILES_FILE", default_profiles)
        profile_name = self.declare_parameter(
            "profile", os.environ.get("GNSS_PROFILE", "nominal")
        ).get_parameter_value().string_value
        self._degrees_per_metre, profile = _load_profile(profiles_file, profile_name)
        self._profile_name = profile_name

        # Run seed offsets the profile seed: different seed_runs of the same
        # scenario sample different noise, while a given seed replays exactly.
        run_seed = int(os.environ.get("SCENARIO_SEED", "0"))
        self._rng = np.random.RandomState(int(profile["seed"]) + run_seed)
        self._sigma_h = float(profile["sigma_h_m"])
        self._sigma_v = float(profile["sigma_v_m"])
        self._bias_east_m = float(profile["bias_east_m"])
        self._bias_north_m = float(profile["bias_north_m"])
        self._dropout_windows = [
            (float(start), float(end)) for start, end in profile.get("dropout_windows_s", [])
        ]

        self._pub = self.create_publisher(
            NavSatFix, "/localization/gnss", qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, "/carla/hero/gnss", self._on_gnss, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"gnss_relay: profile={profile_name} sigma_h={self._sigma_h} "
            f"sigma_v={self._sigma_v} bias=({self._bias_east_m}, {self._bias_north_m}) m "
            f"dropouts={self._dropout_windows} seed={int(profile['seed']) + run_seed}"
        )

    def _dropped(self, sim_time):
        return any(start <= sim_time < end for start, end in self._dropout_windows)

    def _on_gnss(self, msg: NavSatFix) -> None:
        sim_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._dropped(sim_time):
            return

        # Independent horizontal noise on the two axes; sigma_h is the
        # per-axis model the estimator's sigma_xy_m mirrors.
        noise_east = float(self._rng.normal(0.0, self._sigma_h))
        noise_north = float(self._rng.normal(0.0, self._sigma_h))
        noise_alt = float(self._rng.normal(0.0, self._sigma_v))

        out = NavSatFix()
        out.header = msg.header
        out.status = msg.status
        out.latitude = msg.latitude + (self._bias_north_m + noise_north) * self._degrees_per_metre
        out.longitude = msg.longitude + (self._bias_east_m + noise_east) * self._degrees_per_metre
        out.altitude = msg.altitude + noise_alt
        # CARLA ships zero covariance; the estimator's covariance comes from
        # the shared profile (localization.launch.py), not from this message.
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GnssRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
