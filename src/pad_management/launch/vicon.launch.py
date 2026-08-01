from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_padflies(backend: str):
    if backend == "simulation":
        yaml_file = get_package_share_directory("pad_management") + "/config/flies_config_sim.yaml"
    elif backend == "hardware":
        yaml_file = get_package_share_directory("pad_management") + "/config/flies_config_vicon.yaml"

    with open(yaml_file, "r") as file:
        flies = yaml.safe_load(file)["flies"]
        for flie in flies:
            id = flie["id"]
            #if id >= 0xC0:
            #    continue
            yield Node(
                package="padflies_cpp",
                executable="padflie",
                name=f"padflie{id}",
                parameters=[
                    {
                        "id": id,
                        "initial_site": "megapad",
                        "battery_voltage_charged": 4.1,
                    }
                ],
            )

    yield Node(
        package="pad_management_cpp",
        executable="pad_right_provider"
    )

def simulation_group():
    simulation_gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
        #parameters=[{"use_sim_time": True}],
    )
    crazyflies = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie_spawner",
        output="screen",
        parameters=[
            {
                "yaml_path": get_package_share_directory("pad_management")
                + "/config/flies_config_sim.yaml"
            }
        ],
    )

    charging_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "3.14159",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "ChargingBase20",
        ],
    )

    pad_circle = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
           arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "1.0",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "pad_circle",
        ],
    )

    return [simulation_gateway, crazyflies, pad_circle, charging_base]

def hardware_group():

    hardware_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("crazyflie_hardware_bringup"),
             "/launch/hardware.launch.py"]
        ),
        launch_arguments={
            "crazyflie_configuration_yaml": get_package_share_directory(
                "pad_management"
            )
            + "/config/crazyflie_config_vicon.yaml",  # Default params
            "radio_channels": "50, 100",  # Could read from hardware config??
        }.items(),
    )

    motion_caputre = Node(
        package="ros_motioncapture",
        executable="motioncapture_node",
        name="motion_capture",
        output="screen",
        parameters=[
            {
                "type": "vicon",
                "hostname": "172.20.37.201",
                "add_labeled_markers_to_pointcloud": True,
                "topic_name": "pointCloud2",
                "latency_threshold": 0.045,  # 45ms
            }
        ],
    )


    tracker_config = os.path.join(
        get_package_share_directory("pad_management"), "config", "tracker_config.yaml"
    )

    object_tracker = Node(
        package="object_tracker",
        executable="tracker",
        name="tracker",
        parameters=[tracker_config],  # also uses pointCloud2
    )

    point_finder = Node(
        package="pad_management",
        executable="point_finder",
        parameters=[{"point_cloud_topic_name": "pointCloud2"}],
    )

    flies_hardware_yaml = (
        get_package_share_directory("pad_management")
        + "/config/flies_config_vicon.yaml"
    )

    creator = Node(
        package="pad_management",
        executable="pad_creator",
        parameters=[{"padflie_yamls": [flies_hardware_yaml]}],
    )

    pad_circle = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
           arguments=[
            "--x", "0.5",
            "--y", "0.8",
            "--z", "1.0",
            "--yaw", "0",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "ChargingBase20",
            "--child-frame-id", "pad_circle",
        ],
    )

    return [hardware_gateway, motion_caputre, object_tracker, point_finder, creator, pad_circle]

def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="simulation",
        description="Select used backend, choose 'simulation', 'hardware' or 'both'.",
    )
    
    

    hardware_elements = GroupAction(
        actions=hardware_group(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("backend"), "hardware"))
    )
    simulation_elements = GroupAction(
        actions=simulation_group(),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("backend"), "simulation"))
    )
      
    # Broadcast Pads

    pads_hardware_yaml = (
        get_package_share_directory("pad_management")
        + "/config/pads_config_vicon.yaml"
    )

    pads_simulation_yaml = (
        get_package_share_directory("pad_management")
        + "/config/pads_config_sim.yaml"
    )
    pad_broadcaster = Node(
        package="pad_management",
        executable="pad_broadcaster",
        parameters=[
            {"pad_yaml": IfElseSubstitution(
                condition=EqualsSubstitution(LaunchConfiguration("backend"), "hardware"),
                if_value=pads_hardware_yaml,
                else_value=pads_simulation_yaml
            ),
             "pad_size": 0.2,
             "base": "ChargingBase20"}
        ],
    )

    collision_avoidance = Node(
        package="collision_avoidance", executable="collision_avoidance_node"
    )

    velocity_reciprocal_collision_avoidance = Node(
        package="collision_avoidance",
        executable="velocity_reciprocal_collision_avoidance",
    )

    pad_circle = Node(
        package="pad_management",
        executable="pad_land_circle",
        parameters=[{"radius": 1.45, "tf_frame": "pad_circle"}],
    )

    return LaunchDescription(
        [
            backend_arg,
            hardware_elements,
            simulation_elements,
            pad_broadcaster,
            collision_avoidance,
            velocity_reciprocal_collision_avoidance,
            pad_circle,
            OpaqueFunction(
                function=lambda ctxt: generate_padflies(
                    LaunchConfiguration("backend").perform(ctxt),
                )
            ),
        ]
    )
