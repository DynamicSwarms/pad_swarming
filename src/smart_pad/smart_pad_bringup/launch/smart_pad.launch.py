from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    
    smart_pads = []
    smart_pad_tfs = []
    for i in range(2):
        smart_pads.append(
            ComposableNode(
                package='pad_management_cpp',
                plugin='PadRightActionServerNode',
                name=f'smart_pad_{i}',
                parameters=[
                    {
                        'pad_resource_manager_plugin': 'smart_pad::SmartPadResourceManager',
                        'id': i
                    }
                ]
            )
        )

        smart_pad_tfs.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", f"{1.0 + i*0.5}",
                    "--y", "1.2",
                    "--z", "0.05",
                    "--yaw", "0",
                    "--pitch", "0",
                    "--roll", "0",
                    "--frame-id", "world",
                    "--child-frame-id", f"smart_pad_{i}",
                ],
            )
        )

    smart_pad_container = ComposableNodeContainer(
        name='smart_pad_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=smart_pads,
    )


    return LaunchDescription([
        smart_pad_container,
        *smart_pad_tfs,
    ])