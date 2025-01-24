import os

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch.actions import (
    RegisterEventHandler,
    EmitEvent,
    UnregisterEventHandler,
    IncludeLaunchDescription,
)
from launch_ros.event_handlers import OnStateTransition
from launch.events.process import ShutdownProcess
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_agents(context):
    agent_count = LaunchConfiguration("agent_count")
    count = int(agent_count.perform(context))
    for id in range(count):
        yield IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory("agents"),
                    "/launch/agent.launch.py",
                ]
            ),
            launch_arguments={"agent_id": str(id)}.items(),
        )


def generate_launch_description():
    agent_count_launch_arg = DeclareLaunchArgument(
        "agent_count", default_value="2", description="Number of agents"
    )

    # env = Node(
    #    package="agent",
    #    executable="ds_enviroment",
    #    name="ds_enviroment",
    #    output="screen",
    # )

    rviz = Node(package="rviz2", executable="rviz2", name="rviz", output="screen")

    return LaunchDescription(
        [
            # crazyflies,
            agent_count_launch_arg,
            rviz,
            OpaqueFunction(function=generate_agents),
        ]
    )
