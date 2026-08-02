import math
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_scaling_example(context):
    count = int(LaunchConfiguration("count").perform(context))
    spacing = float(LaunchConfiguration("spacing").perform(context))
    columns = max(1, math.ceil(math.sqrt(count)))
    rows = max(1, math.ceil(count / columns))

    flies = []
    for cf_id in range(count):
        column = cf_id % columns
        row = cf_id // columns
        flies.append(
            {
                "id": cf_id,
                "initial_position": [
                    (column - (columns - 1) / 2.0) * spacing,
                    (row - (rows - 1) / 2.0) * spacing,
                    0.0,
                ],
                "initial_orientation": [0.0, 0.0, 0.0, 1.0],
            }
        )

    config = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="crazyflie_scaling_",
        suffix=".yaml",
        delete=False,
    )
    yaml.safe_dump({"flies": flies}, config)
    config.close()

    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
    )
    spawner = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie_spawner",
        output="screen",
        parameters=[{"yaml_path": config.name}],
    )
    controllers = [
        Node(
            package="padflies_cpp",
            executable="padflie",
            name=f"padflie{cf_id}",
            parameters=[
                {
                    "id": cf_id,
                    "initial_site": "",
                    "battery_voltage_charged": 4.1,
                }
            ],
        )
        for cf_id in range(count)
    ]
    avoidance = Node(
        package="collision_avoidance",
        executable="velocity_reciprocal_collision_avoidance",
        parameters=[
            {
                "publish_visualization": ParameterValue(
                    LaunchConfiguration("visualize"), value_type=bool
                )
            }
        ],
        output="screen",
    )
    legacy_avoidance = Node(
        package="collision_avoidance",
        executable="collision_avoidance_node",
        output="screen",
    )
    walker = Node(
        package="crazyflie_flight_controller_examples",
        executable="scaling_controller",
        output="screen",
        parameters=[
            {
                "count": count,
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                "area": ParameterValue(LaunchConfiguration("area"), value_type=float),
                "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
            }
        ],
    )

    return [
        gateway,
        avoidance,
        legacy_avoidance,
        TimerAction(period=0.5, actions=[spawner]),
        *controllers,
        TimerAction(period=2.0, actions=[walker]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("count", default_value="20"),
            DeclareLaunchArgument("spacing", default_value="0.35"),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("area", default_value="5.0"),
            DeclareLaunchArgument("speed", default_value="0.6"),
            DeclareLaunchArgument("visualize", default_value="true"),
            OpaqueFunction(function=launch_scaling_example),
        ]
    )
