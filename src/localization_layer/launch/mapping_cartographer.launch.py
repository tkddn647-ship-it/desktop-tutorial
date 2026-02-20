#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    scan_topic = LaunchConfiguration("scan_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    config_dir = os.path.join(
        get_package_share_directory("localization_layer"),
        "config",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.05",
                description="Resolution of a grid cell in meters",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="1.0",
                description="Occupancy grid publish period in seconds",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry topic",
            ),
            LogInfo(msg="=== localization_layer start: Cartographer Mapping ==="),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("scan", scan_topic),
                    ("imu", imu_topic),
                    ("odom", odom_topic),
                ],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    "cartographer_2d.lua",
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
            ),
        ]
    )
