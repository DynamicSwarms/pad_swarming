from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether to use simulation time",
    )

    sim_clock = Node(
        package="crazyflie_simulation_examples",
        executable="clock",
        output="screen",
        parameters=[{"rate": 10.0}],
        condition=LaunchConfigurationEquals("use_sim_time", "true"),
    )

    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
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

    flies = [0, 1, 7]
    padflies = []
    for i in flies:
        padflies.append(
            Node(
                package="padflies_cpp",
                executable="padflie",
                name=f"padflie{i}",
                parameters=[
                    {
                        "id": i,
                        "pad_id": i,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            )
        )

    position_visualization = Node(
        package="crazyflies",
        executable="position_visualization",
        name="position_visualization",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    pads_config_sim = (
        get_package_share_directory("pad_management") + "/config/pads_config_sim.yaml"
    )
    pad_broadcaster = Node(
        package="pad_management",
        executable="pad_broadcaster",
        parameters=[
            {"pad_yaml": pads_config_sim, "pad_size": 0.2, "base": "ChargingBase20"}
        ],
    )

    collision_avoidance = Node(
        package="collision_avoidance",
        executable="collision_avoidance_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    traffic_controller = Node(
        package="pad_management", executable="pad_traffic_controller"
    )

    charging_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 3.14159 0 0 world ChargingBase20".split(" "),
    )

    pad_circle_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0.5 0.8 1.0 0 0 0 ChargingBase20 pad_circle".split(" "),
    )

    pad_circle = Node(
        package="pad_management",
        executable="pad_land_circle",
        parameters=[{"radius": 1.45, "tf_frame": "pad_circle"}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            sim_clock,
            gateway,
            crazyflies,
            *padflies,
            position_visualization,
            pad_broadcaster,
            collision_avoidance,
            traffic_controller,
            charging_base_tf,
            pad_circle_tf,
            pad_circle,
        ]
    )
