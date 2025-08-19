import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tf2_ros import StaticTransformBroadcaster
from crazyflie_interfaces.msg import PoseStampedArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from pad_management_interfaces.srv import PadPositionReset

from typing import Dict, Tuple
import re

WAIT_FRAMES = 20 # The amount of crazyflie positions waited before it is used as pad_position

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

        self.reset_group = MutuallyExclusiveCallbackGroup()

        self.reset_service = self.create_service(
            srv_type=PadPositionReset,
            srv_name="pad_position_reset",
            callback=self.reset_callback, 
            callback_group=self.reset_group,
        )
    
    def reset_callback(self, request: PadPositionReset.Request, response: PadPositionReset.Response):
        """In case you need to reset the pad position manually. Warning: The next cf position will be
        the new pad position! So maybe don't call this if the cf is currently busy ;)
        
        Args:
            request (PadPositionReset.Request): includes the name
            response (PadPositionReset.Response): success if the action was successfull
        """
        # remove the requested pad from the list, so the code in self.on_cf_positions, will 
        # take the next incoming position of the cf as new pad position.
        if request.all: 
            self.cfs = {}
            self.cfs_wait = {}
            response.success = True
            return response

        if request.pad_name in self.cfs.keys():
            del self.cfs[request.pad_name]
            response.success = True
            return response
        response.success = False
        return response
    

    def pub_tf(self):
        transforms = [] 
        
        keys = list(self.cfs.keys())
        for cf_name in keys: 
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
            elif cf_name not in self.cfs.keys() and self.cfs_wait[cf_name] > WAIT_FRAMES:
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
