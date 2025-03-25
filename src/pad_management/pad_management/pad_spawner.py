import rclpy
from rclpy.node import Node

from tf2_ros import StaticTransformBroadcaster
from crazyflie_interfaces.msg import PoseStampedArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped

from typing import Dict, Tuple
import re


class PadSpawner(Node):
    def __init__(self):
        super().__init__("pad_spawner")

        self.transform_broadcaster = StaticTransformBroadcaster(self)
        self.cf_listener = self.create_subscription(
            msg_type=PoseStampedArray,
            topic="cf_positions",
            callback=self.on_cf_positions,
            qos_profile=10,
        )

        self.cfs: Dict[str, Tuple[Vector3, Quaternion]] = {}
        self.cfs_wait: Dict[str, int] = {}

        self.create_timer(1.0, callback=self.pub_tf)

    def pub_tf(self):
        transforms = []
        for cf_name in self.cfs.keys():
            t = TransformStamped()
            t.transform.translation, t.transform.rotation = self.cfs[cf_name]
            cf_id = int(re.search(r"\d+", cf_name).group())
            t.child_frame_id = f"pad_{cf_id}"
            t.header.frame_id = "world"
            t.header.stamp = self.get_clock().now().to_msg()

            transforms.append(t)
        self.transform_broadcaster.sendTransform(transforms)

    def on_cf_positions(self, cf_positions: PoseStampedArray):
        pose: PoseStamped
        for pose in cf_positions.poses:
            cf_name = pose.header.frame_id
            if cf_name not in self.cfs_wait.keys():
                self.cfs_wait[cf_name] = 0
            elif cf_name not in self.cfs.keys() and self.cfs_wait[cf_name] > 10:
                vec = Vector3()
                vec.x, vec.y, vec.z = [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
                self.cfs[cf_name] = (vec, pose.pose.orientation)
            else:
                self.cfs_wait[cf_name] += 1


def main():
    rclpy.init()

    node = PadSpawner()
    try:
        rclpy.spin(node)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        rclpy.try_shutdown()
    exit()


if __name__ == "__main__":
    main()
