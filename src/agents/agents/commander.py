import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from padflies_interfaces.msg import SendTarget

from typing import Optional


class AgentCommander:

    def __init__(self, node: Node):
        self.__node = node
        self.__qos_profile = 10

        self.__callback_group = MutuallyExclusiveCallbackGroup()

        self.__priority_id = 0

        self.__takeoff_pub: Optional[Publisher] = None
        self.__land_pub: Optional[Publisher] = None
        self.__send_target_pub: Optional[Publisher] = None

    def takeoff(self):
        if self.__takeoff_pub is not None:
            self.__takeoff_pub.publish(Empty())

    def land(self):
        if self.__land_pub is not None:
            self.__land_pub.publish(Empty())

    def send_target(self, pose_stamped: PoseStamped):
        if self.__send_target_pub is not None:
            msg = SendTarget()
            msg.priority_id = self.__priority_id
            msg.target = pose_stamped
            msg.use_yaw = True
            self.__send_target_pub.publish(msg)

    def on_connect(self, prefix: str, priority_id: int):
        self.__priority_id = priority_id
        self.__takeoff_pub = self.__node.create_publisher(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            qos_profile=self.__qos_profile,
            callback_group=self.__callback_group,
        )

        self.__land_pub = self.__node.create_publisher(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            qos_profile=self.__qos_profile,
            callback_group=self.__callback_group,
        )

        self.__send_target_pub = self.__node.create_publisher(
            msg_type=SendTarget,
            topic=prefix + "/send_target",
            qos_profile=self.__qos_profile,
            callback_group=self.__callback_group,
        )

    def on_disconnect(self):
        if self.__takeoff_pub is not None:
            self.__node.destroy_publisher(self.__takeoff_pub)
            self.__takeoff_pub = None
        if self.__land_pub is not None:
            self.__node.destroy_publisher(self.__land_pub)
            self.__land_pub = None
        if self.__send_target_pub is not None:
            self.__node.destroy_publisher(self.__send_target_pub)
            self.__send_target_pub = None
