import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gnss_profile_params(share):
    """Estimator GNSS covariance overrides from the shared profile (#37).

    The scenario GNSS degradation model lives in one file
    (config/gnss_profiles.yaml); the relay applies it to the sensor values
    and the estimator's sigma parameters come from the same profile, so the
    scenario and the filter cannot drift apart.
    """
    profiles_file = os.environ.get(
        "GNSS_PROFILES_FILE", os.path.join(share, "config", "gnss_profiles.yaml")
    )
    profile_name = os.environ.get("GNSS_PROFILE") or "nominal"
    with open(profiles_file, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle)
    profile = doc["profiles"][profile_name]
    return {
        "gnss": {
            "sigma_xy_m": float(profile["sigma_h_m"]),
            "sigma_z_m": float(profile["sigma_v_m"]),
        }
    }


def _launch(context, *args, **kwargs):
    share = get_package_share_directory("brightdrive_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    urdf = os.path.join(share, "urdf", "brightdrive.urdf")
    with open(urdf, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    # Param files default to the installed copies but are overridable from
    # the environment (the compose #26 config-over-code rule: overrides
    # point at files outside the repo, never edits in place).
    localization_params = (
        os.environ.get("LOCALIZATION_PARAMS")
        or os.path.join(share, "config", "localization.yaml")
    )
    ekf_params = os.environ.get("EKF_PARAMS") or os.path.join(share, "config", "ekf.yaml")
    kiss_params = os.environ.get("KISS_PARAMS") or os.path.join(share, "config", "kiss_icp.yaml")

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="brightdrive_bringup",
            executable="imu_contract_covariance.py",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="brightdrive_bringup",
            executable="gnss_relay.py",
            name="gnss_relay",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "profile": os.environ.get("GNSS_PROFILE") or "nominal",
                }
            ],
            output="screen",
        ),
        Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp_node",
            parameters=[kiss_params, {"use_sim_time": use_sim_time}],
            remappings=[("pointcloud_topic", "/carla/hero/lidar")],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_params, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="brightdrive_localization",
            executable="localization_core",
            name="localization_core",
            # The map arrives by subscription (ADR 0003, #34); the georeference
            # and map id live in localization.yaml's map.* parameters. The
            # GNSS sigmas are overridden from the shared profile (#37).
            parameters=[localization_params, _gnss_profile_params(share),
                        {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ]

    if str(rviz).lower() in ("true", "1"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "rviz", "localization.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_launch),
        ]
    )
