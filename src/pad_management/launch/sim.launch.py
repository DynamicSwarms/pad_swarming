from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals

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
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
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

    flies = [0, 1, 2, 3, 4, 5, 6, 7]
    padflies = []
    # for i in flies:
    for i in [0, 1]:
        padflies.append(
            Node(
                package="padflies_cpp",
                executable="padflie",
                name=f"padflie{i}",
                # prefix="gdbserver localhost:3000",
                parameters=[
                    {
                        "id": i,
                        "pad_id": i,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            )
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
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--yaw", "3.14159",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "ChargingBase20",
        ],
    )

    pad_circle_tf = Node(
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
            pad_broadcaster,
            collision_avoidance,
            traffic_controller,
            charging_base_tf,
            pad_circle_tf,
            pad_circle,
        ]
    )
