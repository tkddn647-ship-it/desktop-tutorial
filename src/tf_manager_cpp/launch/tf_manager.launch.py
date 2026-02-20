
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf"
        ),

        Node(
            package="tf_manager_cpp",
            executable="wheel_odom_tf",
            name="wheel_odom_tf"
        ),

    ])
