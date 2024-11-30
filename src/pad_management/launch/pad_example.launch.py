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
        parameters=[{"point_cloud_topic_name": "point_cloud"}],
    )

    motion_caputre = Node(
        package="ros_motioncapture",
        executable="motioncapture_node",
        name="node",
        output="screen",
        parameters=[
            {
                "type": "vicon",
                "hostname": "172.20.37.251",
                "add_labeled_markers_to_pointcloud": True,
                "topic_name": "point_cloud",
            }
        ],
    )

    config = os.path.join(
        get_package_share_directory("object_tracker"), "launch", "tracker_config.yaml"
    )

    object_tracker = Node(
        package="object_tracker",
        executable="tracker",
        name="tracker",
        parameters=[config],
    )

    return LaunchDescription(
        [pad_broadcaster, point_finder, motion_caputre, object_tracker]
    )
