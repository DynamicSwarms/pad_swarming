from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_padflies(pad_flie_config: str):
    file = open(pad_flie_config, "r")
    flies = yaml.safe_load(file)["flies"]
    for flie in flies:
        id = flie["id"]
        channel = flie["channel"]
        pad_id = flie["pad"]
        if id == 161 or id == 162:
            yield Node(
                package="padflies",
                executable="padflie",
                name=f"padflie{id}",
                parameters=[{"id": id, "channel": channel, "pad_id": pad_id}],
            )


def generate_launch_description():
    pad_yaml = (
        get_package_share_directory("pad_management") + "/config/pad_arrangement.yaml"
    )
    padflie_yaml = (
        get_package_share_directory("pad_management")
        + "/config/padflie_arrangement.yaml"
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
                "topic_name": "pointCloud",
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

    creator = Node(package="pad_management", executable="pad_creator")

    return LaunchDescription(
        [
            creator,
            pad_broadcaster,
            point_finder,
            motion_caputre,
            object_tracker,
            OpaqueFunction(function=lambda ctxt: generate_padflies(padflie_yaml)),
        ]
    )
