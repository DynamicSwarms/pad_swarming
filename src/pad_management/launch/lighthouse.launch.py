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
                        "battery_voltage_empty": 3.30,
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

    setup_yaml_arg = DeclareLaunchArgument(
        "setup_yaml",
        default_value=get_package_share_directory("pad_management")
        + "/config/lighthouse_config.yaml",
        description="Select a .yaml describing your system setup. (Crazyflie-IDs, Channels, Pads)",
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
            "crazyflie_configuration_yaml": get_package_share_directory(
                "pad_management"
            )
            + "/config/crazyflie_config_lh.yaml",
            "radio_channels": "50, 100",  # Could read from hardware config??
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
        executable="gui_creator",
        parameters=[
            {
                "setup_yaml": LaunchConfiguration("setup_yaml"),
                "backend": LaunchConfiguration("backend"),
            }
        ],
    )

    gui_state = Node(
        package="pad_management",
        executable="gui_state",
        parameters=[{"setup_yaml": LaunchConfiguration("setup_yaml")}],
    )

    pad_spawner = Node(
        package="pad_management",
        executable="pad_spawner",
    )

    collision_avoidance = Node(
        package="collision_avoidance", executable="collision_avoidance_node"
    )

    traffic_controller = Node(
        package="pad_management", executable="pad_traffic_controller"
    )

    pad_circle = Node(
        package="pad_management",
        executable="pad_land_circle",
        parameters=[{"radius": 0.3}],
    )

    pad_circle_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="-0.5 0.6 0.5 0 0 0 world pad_circle".split(" "),
    )
    return LaunchDescription(
        [
            backend_arg,
            setup_yaml_arg,
            webots_gateway,
            hardware_gateway,
            position_visualization,
            creator,
            pad_spawner,
            OpaqueFunction(
                function=lambda ctxt: generate_padflies(
                    LaunchConfiguration("setup_yaml").perform(ctxt),
                    LaunchConfiguration("backend").perform(ctxt),
                )
            ),
            collision_avoidance,
            traffic_controller,
            pad_circle,
            pad_circle_tf,
            gui_state,
        ]
    )
