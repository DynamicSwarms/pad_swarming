from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pad_yaml = (
        get_package_share_directory("pad_management") + "/launch/pad_arrangement.yaml"
    )

    pad_broadcaster = Node(
        package="pad_management",
        executable="pad_broadcaster",
        parameters=[{"pad_yaml": pad_yaml, "pad_size": 0.2, "base": "ChargingBase20"}],
    )

    point_finder = Node(
        package="pad_management",
        executable="point_finder",
        parameters=[{"point_cloud_topic_name": "pointCloud"}],
    )

    return LaunchDescription([pad_broadcaster, point_finder])
