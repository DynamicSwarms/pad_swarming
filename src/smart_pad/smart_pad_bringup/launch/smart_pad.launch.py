from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    smart_pads = []
    smart_pad_tfs = []
    for i in range(2):
        smart_pads.append(
            Node(
                package='smart_pad',
                executable='smart_pad',
                name=f'smart_pad_{i}',
                output='screen',
                parameters=[
                    {"id": i}
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


    return LaunchDescription([
        *smart_pads,
        *smart_pad_tfs,
    ])