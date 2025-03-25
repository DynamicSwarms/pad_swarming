from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_padflies(lighthouse_yaml: str, backend: str):
    with open(lighthouse_yaml, "r") as file:
        flies = yaml.safe_load(file)["flies"]
        for flie in flies:
            id = flie["id"]
            channel = flie["channel"] if backend == "hardware" else 0
            pad_id = flie["pad"]

            yield Node(
                package="padflies",
                executable="padflie",
                name=f"padflie{id}",
                parameters=[
                    {
                        "id": id,
                        "channel": channel,
                        "pad_id": pad_id,
                        "type": backend,
                    }
                ],
            )


def generate_launch_description():
    webots_gateway_dir = get_package_share_directory("crazyflie_webots_gateway")
    hardware_gateway_dir = get_package_share_directory("crazyflie_hardware_gateway")

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="webots",
        description="Select used backend, choose 'webots', 'hardware' or 'both'.",
    )

    lighthouse_yaml = (
        get_package_share_directory("pad_management") + "/config/lighthouse_config.yaml"
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
        launch_arguments={
            "radio_channels": "50",  # Could read from hardware config??
        }.items(),
    )

    position_visualization = Node(
        package="crazyflies",
        executable="position_visualization",
        name="position_visualization",
    )

    ## Custom stuff

    creator = Node(
        package="pad_management",
        executable="default_creator",
        parameters=[{"setup_yaml": lighthouse_yaml}],
    )

    pad_spawner = Node(
        package="pad_management",
        executable="pad_spawner",
    )

    traffic_controller = Node(
        package="pad_management", executable="pad_traffic_controller"
    )

    pad_circle = Node(package="pad_management", executable="pad_land_circle")

    # For webots we need ChargingBase in tf
    pad_circle_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 1.2 0 0 0 world pad_circle".split(" "),
    )
    return LaunchDescription(
        [
            backend_arg,
            webots_gateway,
            hardware_gateway,
            position_visualization,
            creator,
            pad_spawner,
            OpaqueFunction(
                function=lambda ctxt: generate_padflies(
                    lighthouse_yaml, LaunchConfiguration("backend")
                )
            ),
            traffic_controller,
            pad_circle,
            pad_circle_tf,
        ]
    )
