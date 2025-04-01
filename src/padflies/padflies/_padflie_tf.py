import rclpy
from rclpy.node import Node
import rclpy.duration
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_py
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped, Quaternion
import tf2_geometry_msgs as tf2_geom
import tf_transformations

from crazyflie_interfaces_python.positions import CfPositionBuffer
from typing import Optional, Tuple, List, Callable


### This needs to be ported to crazyflie_interfaces_python
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
from crazyflie_interfaces.msg import PoseStampedArray


class CfPositionListener:
    def __init__(self, buffer: CfPositionBuffer, node: Node):
        self._buffer = buffer
        self._node = node
        self._callback_group = MutuallyExclusiveCallbackGroup()

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cf_position_sub = node.create_subscription(
            msg_type=PoseStampedArray,
            topic="/cf_positions",
            callback=self.callback,
            qos_profile=qos,
            callback_group=self._callback_group,
        )

    def __del__(self) -> None:
        self.unregister()

    def unregister(self):
        self._node.destroy_subscription(self._cf_position_sub)

    def callback(self, msg: PoseStampedArray):
        pose_stamped: PoseStamped
        for pose_stamped in msg.poses:
            self._buffer.set_position(pose_stamped, msg.header.frame_id)


class PadflieTF:

    def __init__(
        self,
        node: Node,
        sleep: Callable[[float], None],
        pad_name: str,
        cf_name: str,
        world: str = "world",
    ):
        self._node = node
        self._sleep = sleep
        self._pad_name = pad_name
        self._cf_name = cf_name
        self._world = world

        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=1))
        self._cf_buffer = CfPositionBuffer(node=node)

    def activate(self):
        self._tf_listener = TransformListener(self._tf_buffer, node=self._node)
        self._cf_listener = CfPositionListener(self._cf_buffer, node=self._node)

    def deactivate(self):
        self._tf_listener.unregister()
        self._cf_listener.unregister()

        del self._tf_listener
        del self._cf_listener

    def get_pad_position_or_timeout(
        self, timeout_sec: float
    ) -> Tuple[List[float], float]:
        """Retrieves the pad position or timess out after given time.

        Args:
            timeout_sec (float): The maximum time to wait for the pad position to become available.

        Raises:
            TimeoutError: If timeout occurs

        Returns:
            Tuple[List[float], float]: The position and yaw of the pad in world coordinates.
        """
        pad_position_and_yaw = self.get_pad_position_and_yaw()
        while pad_position_and_yaw is None and timeout_sec > 0.0:
            self._sleep(0.1)
            timeout_sec -= 0.1
            pad_position_and_yaw = self.get_pad_position_and_yaw()
        if pad_position_and_yaw is None:
            raise TimeoutError(f"Pad with name: {self._pad_name}, could not be found")
        return pad_position_and_yaw

    def get_pad_position_and_yaw(self) -> Optional[Tuple[List[float], float]]:
        """Gets the position of our pad.

        Returns:
            Optional[Tuple[List[float], float]]: None if not possible. Otherwise position and yaw of pad in world coords.
        """

        t = self.get_transform(
            target_frame=self._world,
            source_frame=self._pad_name,
        )
        if t is None:
            return None

        position = [
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ]
        quaternion = (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        )
        _roll, _pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        return (position, yaw)

    def get_pad_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._pad_name
        return pose

    def get_pad_pose_world(self) -> Optional[PoseStamped]:
        transform = self.get_transform(
            target_frame=self._world,
            source_frame=self._pad_name,
        )
        if transform is None:
            return None
        zero_pose = PoseStamped()
        zero_pose.header.frame_id = self._world
        return tf2_geom.do_transform_pose_stamped(zero_pose, transform)

    def get_cf_pose_stamped(
        self,
        frame_id: Optional[str],
        world_quat: Quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    ) -> Optional[PoseStamped]:
        """Get the pose of the cf in a specifc frame_id
        Args:
            frame_id (Optional[str], optional): The frame to get the cf pose in. Defaults to None.
            world_quat (Quaternion, optional): A Quaternion which is added to the pose. Defaults to Quaternion(x=0.0, y=0.0, z=0.0, w=1.0).

        Returns:
            Optional[PoseStamped]: A Pose Stamped with frame_id as reference.
        """
        transform = self.get_transform(target_frame=frame_id, source_frame=self._world)
        world_pose = self._cf_buffer.get_position(self._cf_name)

        if transform is None or world_pose is None:
            return None
        return tf2_geom.do_transform_pose_stamped(world_pose, transform)

    def get_cf_position(self) -> Optional[List[float]]:
        """Gets the position of the crazyflie in world coordinates.

        Returns:
            Optional[List[float]]: The Position of the cf in world coordinates.
        """
        world_pose = self._cf_buffer.get_position(self._cf_name)
        if world_pose is None:
            return None
        return [
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            world_pose.pose.position.z,
        ]

    def transform_point_stamped(
        self,
        point: PointStamped,
        target_frame: str,
    ) -> Optional[PointStamped]:
        """Transforms stamped point into another frame.

        The source frame is in the stamp of the point.

        Args:
            point (PointStamped): The point to transform (header frame_id is used as source frame).
            target_frame (str): The frame the point should get transformed to.
        Returns:
            Optional[PointStamped]: The transformed point. None if was not possible.
        """
        transform = self.get_transform(
            target_frame=target_frame, source_frame=point.header.frame_id
        )
        if transform is None:
            return None

        return tf2_geom.do_transform_point(point=point, transform=transform)

    def transform_pose_stamped(
        self, pose: PoseStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        """Transforms a pose from the frame given in pose stamp to the target frame

        Similar to transform point stamped

        Args:
            pose (PoseStamped): The stamped pose to transform
            target_frame (str): The target frame to transform the pose to

        Returns:
            Optional[PoseStamped]: The new pose in the target frame
        """
        transform = self.get_transform(
            target_frame=target_frame, source_frame=pose.header.frame_id
        )
        if transform is None:
            return None

        return tf2_geom.do_transform_pose_stamped(pose=pose, transform=transform)

    def pose_stamped_to_world_position_and_yaw(
        self, pose: PoseStamped
    ) -> Optional[Tuple[List[float], float]]:
        world_pose = self.transform_pose_stamped(pose, self._world)
        if world_pose is None:
            return None
        _roll, _pitch, yaw = tf_transformations.euler_from_quaternion(
            (
                world_pose.pose.orientation.x,
                world_pose.pose.orientation.y,
                world_pose.pose.orientation.z,
                world_pose.pose.orientation.w,
            )
        )
        position = [
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            world_pose.pose.position.z,
        ]
        return (position, yaw)

    def get_transform(
        self, target_frame: str, source_frame: str
    ) -> Optional[TransformStamped]:
        try:
            time = rclpy.time.Time(seconds=0, nanoseconds=0)
            # The core check needs to be done.
            # Otherwise the lookup transform timeout gets ignored
            if not self._tf_buffer.can_transform_core(target_frame, source_frame, time):
                return None

            return self._tf_buffer.lookup_transform_core(
                target_frame=target_frame,
                source_frame=source_frame,
                time=time,
            )

        except (tf2_py.LookupException, tf2_py.ConnectivityException) as ex:
            # This is most likely a Transform execption, which is expected and therefore None is ok.
            # It is very likely
            return None
