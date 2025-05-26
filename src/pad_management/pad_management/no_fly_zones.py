import time
import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState
from padflies_interfaces.srv import GetBBoxes
from padflies_interfaces.msg import BBox
from ament_index_python.packages import get_package_share_directory

import yaml
import numpy as np
from itertools import cycle
from threading import Lock
from .creator import Creator, BackendType
from std_msgs.msg import Empty

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped, Vector3


class NoFlyZoneManager(Node):
    def __init__(self):
        super().__init__("no_fly_zone_manager")
        
        self.declare_parameter("bboxes_yaml", "")

        self.service = self.create_service(
            srv_type=GetBBoxes,
            srv_name="get_no_fly_zones",
            callback=self.callback, 
            callback_group=ReentrantCallbackGroup()
        )
        self.bboxes = []

        _bbox_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._update_pub = self.create_publisher(
            Empty,
            "/update_bboxes",
            qos_profile=_bbox_qos_profile,
        )

        self.update_bboxes()

    def callback(self, request: GetBBoxes.Request, response: GetBBoxes.Response):
        response.bboxes = []
        self.get_logger().error(str(self.bboxes))
        for box in self.bboxes:
            bbox_msg = BBox()
            bbox_msg.transform = create_transform_stamped(
                translation = [float(x) for x in box.get("center", [0.0, 0.0, 0.0])],
                rotation = [float(x) for x in box.get("rotation", [0.0, 0.0, 0.0, 1.0])],
                parent_frame = str(box.get("parent_frame", "world")),
                child_frame = str(box.get("name", "unnamed_box")),
                time_stamp = self.get_clock().now().to_msg()
            )
            size_list = [float(x) for x in box.get("size", [1.0, 1.0, 1.0])]
            bbox_msg.size = Vector3(x=size_list[0], y=size_list[1], z=size_list[2])

            self.get_logger().error(str(bbox_msg))
            response.bboxes.append(bbox_msg)
        return response
    
    def update_bboxes(self):
        file_yaml = self.get_parameter("bboxes_yaml").get_parameter_value().string_value

        with open(file_yaml, "r") as file:
            data = yaml.safe_load(file)
            #TODO format
            self.bboxes = data.get("bboxes")
        
        while self._update_pub.get_subscription_count() == 0:
            time.sleep(0.5)
        
        self._update_pub.publish(Empty())


def main():
    rclpy.init()

    manager = NoFlyZoneManager()
    rclpy.spin(manager)

if __name__ == "__main__":
    main()


def create_transform_stamped(translation, rotation, parent_frame, child_frame, time_stamp=None):
    """
    Create a TransformStamped message from position and orientation lists.
    
    Args:
        translation (list): [x, y, z]
        rotation (list): [x, y, z, w] (quaternion)
        parent_frame (str): name of the parent frame
        child_frame (str): name of the child frame
        time_stamp (builtin_interfaces.msg.Time, optional): timestamp
    
    Returns:
        TransformStamped
    """
    t = TransformStamped()
    
    # Use current time if not provided
    if time_stamp is None:
        from builtin_interfaces.msg import Time
        t.header.stamp = Time()  # This should usually be set from a clock or node.get_clock().now().to_msg()
    else:
        t.header.stamp = time_stamp
    
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    
    return t
