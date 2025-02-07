import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from padflies_interfaces.msg import SendTarget, PadflieInfo

from typing import Optional, List


class AgentCommander:

    def __init__(self, node: Node):
        self.__node = node
        self.__qos_profile = 10

        self.__callback_group = MutuallyExclusiveCallbackGroup()

        self.__priority_id = 0

        self.__takeoff_pub: Optional[Publisher] = None
        self.__land_pub: Optional[Publisher] = None
        self.__send_target_pub: Optional[Publisher] = None

        self.__info_sub: Optional[Subscription] = None

        self._pose_info: Optional[PoseStamped] = None
        self._pose_world_info: Optional[Pose] = None

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

    def get_world_positon(self) -> Optional[List[float]]:
        if self._pose_world_info is None:
            return None

        return [
            self._pose_world_info.position.x,
            self._pose_world_info.position.y,
            self._pose_world_info.position.z,
        ]

    def get_local_position(self) -> Optional[List[float]]:
        if self._pose_info is None:
            return None
        return [
            self._pose_info.pose.position.x,
            self._pose_info.pose.position.y,
            self._pose_info.pose.position.z,
        ]

    def __info_callback(self, info: PadflieInfo):
        if info.pose_valid:
            self._pose_info = info.pose
        if info.pose_world_valid:
            self._pose_world_info = info.pose_world

    def on_connect(self, prefix: str, priority_id: int):
        self._pose_info = None
        self._pose_world_info = None
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

        self.__info_sub = self.__node.create_subscription(
            msg_type=PadflieInfo,
            topic=prefix + "/info",
            callback=self.__info_callback,
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
        if self.__info_sub is not None:
            self.__node.destroy_subscription(self.__info_sub)
            self.__info_sub = None
