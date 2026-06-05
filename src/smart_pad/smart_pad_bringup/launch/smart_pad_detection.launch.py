from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_capture = Node(
        package="ros_motioncapture",
        executable="motioncapture_node",
        name="motion_capture",
        output="screen",
        parameters=[
            {
                "type": "vicon",
                "hostname": "172.20.37.251",
                "add_labeled_markers_to_pointcloud": True,
                "topic_name": "pointCloud2",
                "latency_threshold": 0.045,
            }
        ],
    )

    id_finder_node = Node(
        package="smart_pad_detection",
        executable="ID_finder_node",
        name="id_finder_node",
        output="screen",
        parameters=[
            {
                "pc2_topic": "pointCloud2",
                "radius": 0.01,
                "latency_threshold": 0.045,
            }
        ],
    )

    id_sort_node = Node(
        package="smart_pad_detection",
        executable="ID_sort_node",
        name="id_sort_node",
        output="screen",
    )

    return LaunchDescription(
        [
            motion_capture,
            id_finder_node,
            id_sort_node,
        ]
    )
