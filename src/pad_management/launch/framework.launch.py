import os

from launch import LaunchDescription, LaunchContext

from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("crazyflies")

    webots_dir = get_package_share_directory("crazyflie_webots_gateway")
    hardware_dir = get_package_share_directory("crazyflie_hardware_gateway")

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="webots",
        description="Select used backend, choose 'webots', 'hardware' or 'both'.",
    )

    start_hardware = LaunchConfigurationNotEquals("backend", "webots")
    start_webots = LaunchConfigurationNotEquals("backend", "hardware")
    # This doesnt look too clean. In Jazzy we can use Substitions with Equals and Or

    webots_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([webots_dir, "/launch/gateway.launch.py"]),
        condition=start_webots,
        launch_arguments={}.items(),
    )

    hardware_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [hardware_dir, "/launch/crazyflie_hardware_gateway.launch.py"]
        ),
        condition=start_hardware,
    )

    motion_caputre = Node(
        condition=start_hardware,
        package="ros_motioncapture",
        executable="motioncapture_node",
        name="node",
        output="screen",
        parameters=[
            {
                "type": "vicon",
                "hostname": "172.20.37.251",
                "add_labeled_markers_to_pointcloud": True,
            }
        ],
    )

    config = os.path.join(
        get_package_share_directory("object_tracker"), "launch", "tracker_config.yaml"
    )

    object_tracker = Node(
        condition=start_hardware,
        package="object_tracker",
        # namespace='object_tracker',
        executable="tracker",
        name="tracker",
        parameters=[config],
    )

    return LaunchDescription(
        [backend_arg, webots_gateway, hardware_gateway, motion_caputre, object_tracker]
    )
