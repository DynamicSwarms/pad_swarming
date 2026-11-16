def generate_launch_description():
    from launch import LaunchDescription
    from launch_ros.actions import Node

    pad_right_test_nodes = [
        Node(
            package="pad_management_examples",
            executable="pad_right_test",
            name=f"pad_right_test_{i}",
            parameters=[{"id": i, "hold_time": i * 2.0}],
        )
        for i in range(1, 4)
    ]

    return LaunchDescription(
        [
            *pad_right_test_nodes,
        ]
    )
