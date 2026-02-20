#!/usr/bin/env python3
"""
localization_layer.launch.py
────────────────────────────
Wheel odom + IMU를 사용하는 2D EKF 로컬라이제이션 통합 실행

포함 노드:
  1. robot_localization/ekf_node — /odom + /imu/data 융합

사용법:
  ros2 launch localization_layer localization_layer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    ekf_config_path = os.path.join(
        get_package_share_directory("localization_layer"),
        "config",
        "ekf_localization.yaml",
    )

    return LaunchDescription(
        [
            # ── Arguments ──────────────────────────────────────────────
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="시뮬레이터 사용 여부",
            ),

            LogInfo(msg="=== localization_layer 시작: EKF(Localization) ==="),

            # ── EKF 로컬라이제이션 노드 ───────────────────────────────
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_localization_node",
                output="screen",
                parameters=[
                    ekf_config_path,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )

