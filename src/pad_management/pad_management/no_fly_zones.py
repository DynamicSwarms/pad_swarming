import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState
from padflies_interfaces.srv import GetBBoxes
from ament_index_python.packages import get_package_share_directory

import yaml
import numpy as np
from itertools import cycle
from threading import Lock
from .creator import Creator, BackendType
from std_msgs.msg import Empty

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


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

    def callback(self, request: GetBBoxes.Request, response: GetBBoxes.Response):
        response.bboxes = []
        return response
    
    def update_bboxes(self):
        file_yaml = self.get_parameter("bboxes_yaml").get_parameter_value().string_value

        with open(file_yaml, "r") as file:
            data = yaml.safe_load(file)
            #TODO format
            self.bboxes = data.get("bboxes")
            print("ROSSI", self.bboxes)
        self._update_pub.publish(Empty())


def main():
    rclpy.init()

    manager = NoFlyZoneManager()
    rclpy.spin(manager)

if __name__ == "__main__":
    main()