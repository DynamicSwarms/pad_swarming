from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_padflies(flies_hardware_yaml: str, flies_webots_yaml: str):
    yamls = {"webots": flies_webots_yaml, "hardware": flies_hardware_yaml}

    for cf_type in yamls.keys():
        with open(yamls[cf_type], "r") as file:
            flies = yaml.safe_load(file)["flies"]

            for flie in flies:
                id = flie["id"]
                channel = flie["channel"] if cf_type == "hardware" else 0
                pad_id = flie["pad"]
                if (
                    cf_type == "webots"
                    or id == 161
                    or id == 162
                    or id == 166
                    or id == 167
                ):
                    yield Node(
                        package="padflies",
                        executable="padflie",
                        name=f"padflie{id}",
                        parameters=[
                            {
                                "id": id,
                                "channel": channel,
                                "pad_id": pad_id,
                                "type": cf_type,
                            }
                        ],
                    )


def generate_launch_description():
    webots_gateway_dir = get_package_share_directory("crazyflie_webots_gateway")
    hardware_gateway_dir = get_package_share_directory("crazyflie_hardware_gateway")
    webots_connector_dir = get_package_share_directory("webots_connector")

    pads_hardware_yaml = (
        get_package_share_directory("pad_management")
        + "/config/pads_config_hardware.yaml"
    )
    flies_hardware_yaml = (
        get_package_share_directory("pad_management")
        + "/config/flies_config_hardware.yaml"
    )

    tracker_config = os.path.join(
        get_package_share_directory("pad_management"), "config", "tracker_config.yaml"
    )

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="webots",
        description="Select used backend, choose 'webots', 'hardware' or 'both'.",
    )

    start_hardware = LaunchConfigurationNotEquals("backend", "webots")
    start_webots = LaunchConfigurationNotEquals("backend", "hardware")
    # This doesnt look too clean. In Jazzy we can use Substitions with Equals and Or

    webots_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [webots_gateway_dir, "/launch/gateway.launch.py"]
        ),
        condition=start_webots,
        launch_arguments={}.items(),
    )

    hardware_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [hardware_gateway_dir, "/launch/crazyflie_hardware_gateway.launch.py"]
        ),
        condition=start_hardware,
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
                "topic_name": "pointCloud2",
            }
        ],
        # condition=start_hardware,
    )

    object_tracker = Node(
        package="object_tracker",
        executable="tracker",
        name="tracker",
        parameters=[tracker_config],  # also uses pointCloud2
        condition=start_hardware,
    )

    ## Custom stuff

    # Webots connector
    connector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [webots_connector_dir, "/launch/webots_connector.launch.py"]
        ),
        launch_arguments={"pointcloud_topic_name": "sim_cloud"}.items(),
        condition=start_webots,
    )

    pointcloud_combiner = Node(
        package="pad_management",
        executable="pointcloud_combiner",
        parameters=[
            {
                "input_names": ["sim_cloud", "pointCloud2"],
                "output_name": "combined_cloud",
            }
        ],
    )

    # webots pads get broadcasted by the connector itself
    pad_broadcaster = Node(
        package="pad_management",
        executable="pad_broadcaster",
        parameters=[
            {"pad_yaml": pads_hardware_yaml, "pad_size": 0.2, "base": "ChargingBase20"}
        ],
        condition=start_hardware,
    )

    point_finder = Node(
        package="pad_management",
        executable="point_finder",
        parameters=[{"point_cloud_topic_name": "combined_cloud"}],
    )
    flies_webots_yaml = os.path.join(
        webots_connector_dir, "config", "padflies_config.yaml"
    )
    creator = Node(
        package="pad_management",
        executable="pad_creator",
        parameters=[{"padflie_yamls": [flies_hardware_yaml, flies_webots_yaml]}],
    )

    return LaunchDescription(
        [
            backend_arg,
            webots_gateway,
            hardware_gateway,
            motion_caputre,
            object_tracker,
            connector,
            pointcloud_combiner,
            pad_broadcaster,
            creator,
            point_finder,
            OpaqueFunction(
                function=lambda ctxt: generate_padflies(
                    flies_hardware_yaml, flies_webots_yaml
                )
            ),
        ]
    )
