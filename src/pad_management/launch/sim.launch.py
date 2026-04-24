from launch_ros.actions import Node
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
        parameters=[{"use_sim_time": False}],
    )

    flies_count = 3
    crazyflies = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie_spawner",
        output="screen",
        parameters=[{"count": flies_count}],
    )
    padflies = []
    for i in range(flies_count):
        padflies.append(
            Node(
                package="padflies_cpp",
                executable="padflie",
                name=f"padflie{i}",
                parameters=[{"id": i, "pad_id": i}],
            )
        )


    position_visualization = Node(
        package="crazyflies",
        executable="position_visualization",
        name="position_visualization",
    )

    pads_config_sim = (
        get_package_share_directory("pad_management")
        + "/config/pads_config_sim.yaml"
    )
    pad_broadcaster = Node(
        package="pad_management",
        executable="pad_broadcaster",
        parameters=[
            {"pad_yaml": pads_config_sim, "pad_size": 0.2, "base": "ChargingBase20"}
        ],
    )

    collision_avoidance = Node(
        package="collision_avoidance", executable="collision_avoidance_node"
    )

    traffic_controller = Node(
        package="pad_management", executable="pad_traffic_controller"
    )

    charging_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 world ChargingBase20".split(" "),
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
