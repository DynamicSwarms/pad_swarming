from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    EmitEvent,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.events.process import ShutdownProcess
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration


def create_agent(context):
    agent_id = LaunchConfiguration("agent_id")
    id = int(agent_id.perform(context))
    agent = Node(
        package="agents",
        executable="wand_agent",
        name="agent{}".format(id),
        namespace="",
        parameters=[{"id": id}],
    )

    # As the agent is a LifecycleNode an event handler can be registered to the transition from shuttingdown to finalized
    # but creating a lifecycle node also creates a node which provides services for the nodes transition
    # this node however can not be killed. This makes it absolutely unsuitable for our purposes.

    yield agent


def generate_launch_description():
    agent_id_launch_arg = DeclareLaunchArgument(
        "agent_id", default_value="0", description="The id of the agent spawned."
    )

    return LaunchDescription(
        [agent_id_launch_arg, OpaqueFunction(function=create_agent)]
    )
