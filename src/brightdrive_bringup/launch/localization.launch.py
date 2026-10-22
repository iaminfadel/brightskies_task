import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    share = get_package_share_directory("brightdrive_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_dir = LaunchConfiguration("map_dir")
    rviz = LaunchConfiguration("rviz")

    urdf = os.path.join(share, "urdf", "brightdrive.urdf")
    with open(urdf, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    localization_params = os.path.join(share, "config", "localization.yaml")
    ekf_params = os.path.join(share, "config", "ekf.yaml")
    kiss_params = os.path.join(share, "config", "kiss_icp.yaml")

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
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp_node",
            parameters=[kiss_params, {"use_sim_time": use_sim_time}],
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
            parameters=[localization_params, {"use_sim_time": use_sim_time, "map.dir": map_dir}],
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
            DeclareLaunchArgument("map_dir", default_value="/ws/maps/current/town05-v1"),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(function=_launch),
        ]
    )
