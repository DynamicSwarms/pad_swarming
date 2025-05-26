import math
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
import copy
import numpy as np
from itertools import cycle
from threading import Lock

import tf2_ros
from .creator import Creator, BackendType
from std_msgs.msg import Empty

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped, Vector3, Point
from tf_transformations import translation_matrix, quaternion_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix
from visualization_msgs.msg import Marker


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
        self.bboxes_og = []
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

        self.box_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)  # assuming rclpy node

        self.br = tf2_ros.TransformBroadcaster(self)        
        self.timer = self.create_timer(0.05, self.publish_transform)  # 20 Hz
        self.angle = 0.0


        self.initialize_boxes()
        self.timer = self.create_timer(0.1, self.update_bboxes)
        self.timer = self.create_timer(0.1, self.publish_boxes)

    def publish_transform(self):
        radius = 2.0
        speed = 0.5  # radians per second
        self.angle += speed * 0.05  # 0.05 is the timer period

        x = radius * math.cos(self.angle)
        y = radius * math.sin(self.angle)
        z = 0.5

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'rotating_frame'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)
        
    def callback(self, request: GetBBoxes.Request, response: GetBBoxes.Response):
        response.bboxes = []

        for bbox_msg in self.bboxes:
            response.bboxes.append(bbox_msg)
        return response
    
    def initialize_boxes(self):
        file_yaml = self.get_parameter("bboxes_yaml").get_parameter_value().string_value

        with open(file_yaml, "r") as file:
            data = yaml.safe_load(file)
            bboxes = data.get("bboxes")
            for box in bboxes:
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
                self.bboxes_og.append(bbox_msg)
        self.update_bboxes()
    

    def update_bboxes(self):
        # convert all transforms to world coordinates
        world_frame = 'world'
        bboxes = []

        for obj in self.bboxes_og:
            bbox_msg = copy.deepcopy(obj)
            child_frame_id = bbox_msg.transform.child_frame_id
            parent_frame_id = bbox_msg.transform.header.frame_id

            try:
                tf_world_to_parent = self.buffer.lookup_transform(
                    target_frame=world_frame,
                    source_frame=parent_frame_id,
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0)
                )

                #self.get_logger().error(str(tf_world_to_parent.transform.translation))
                # Convert both to matrices
                T_world_to_parent = transform_to_matrix(tf_world_to_parent)
                T_parent_to_child = transform_to_matrix(bbox_msg.transform)

                # Multiply: T_world_to_child = T_world_to_parent @ T_parent_to_child
                T_world_to_child = T_world_to_parent @ T_parent_to_child
                world_to_child = matrix_to_transform(
                    T_world_to_child,
                    parent=parent_frame_id,
                    child=child_frame_id,
                    stamp=self.get_clock().now().to_msg()
                )

                bbox_msg.transform = world_to_child
                bboxes.append(bbox_msg)
            except tf2_ros.LookupException as e:
                self.get_logger().warn(str(e))

        self.bboxes = bboxes
        if self._update_pub.get_subscription_count() == 0:
            return
        self._update_pub.publish(Empty())

    def publish_boxes(self):
        for i, box in enumerate(self.bboxes):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = box.transform.transform.translation.x
            marker.pose.position.y = box.transform.transform.translation.y
            marker.pose.position.z = box.transform.transform.translation.z

            # Orientation (identity quaternion)
            marker.pose.orientation.x = box.transform.transform.rotation.x
            marker.pose.orientation.y = box.transform.transform.rotation.y
            marker.pose.orientation.z = box.transform.transform.rotation.z
            marker.pose.orientation.w = box.transform.transform.rotation.w

            # Size (scale)
            marker.scale.x = box.size.x  # Width
            marker.scale.y = box.size.y  # Depth
            marker.scale.z = box.size.z  # Height

            # Color (RGBA)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.box_publisher.publish(marker)

def main():
    rclpy.init()
    manager = NoFlyZoneManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

def matrix_to_transform(matrix, parent, child, stamp):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.header.stamp = stamp
    trans = translation_from_matrix(matrix)
    quat = quaternion_from_matrix(matrix)
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t

def transform_to_matrix(t: TransformStamped):
    trans = t.transform.translation
    rot = t.transform.rotation
    T = concatenate_matrices(
        translation_matrix([trans.x, trans.y, trans.z]),
        quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    )
    return T


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
