import rclpy
import rclpy.time
import tf2_py
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped

from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped
import tf_transformations
import math

from typing import Optional, Tuple, List, Callable


class PadflieTF:

    def __init__(
        self,
        tf_buffer: Buffer,
        sleep: Callable[[float], None],
        pad_name: str,
        world: str = "world",
    ):
        self.__tf_buffer = tf_buffer
        self.__sleep = sleep
        self.__pad_name = pad_name
        self.__world = world

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
            self.__sleep(0.1)
            timeout_sec -= 0.1
            pad_position_and_yaw = self.get_pad_position_and_yaw()
        if pad_position_and_yaw is None:
            raise TimeoutError(f"Pad with name: {self.__pad_name}, could not be found")
        return pad_position_and_yaw

    def get_pad_position_and_yaw(self) -> Optional[Tuple[List[float], float]]:
        """Gets the position of our pad.

        Returns:
            Optional[Tuple[List[float], float]]: None if not possible. Otherwise position and yaw of pad in world coords.
        """

        t = self.get_transform(
            target_frame=self.__world,
            source_frame=self.__pad_name,
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
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        if yaw < 0:
            yaw += math.pi
        else:
            yaw -= math.pi
        return (position, yaw)

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

        return do_transform_point(point=point, transform=transform)

    def transform_pose_stamped(
        self, pose: PoseStamped, target_frame: str, logger
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
        logger.info(str(transform))
        if transform is None:
            return None

        return do_transform_pose_stamped(pose=pose, transform=transform)

    def get_transform(
        self, target_frame: str, source_frame: str
    ) -> Optional[TransformStamped]:
        try:
            # The core check needs to be done.
            # Otherwise the lookup transform timeout gets ignored
            if not self.__tf_buffer.can_transform_core(
                target_frame, source_frame, rclpy.time.Time()
            ):
                return None

            return self.__tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time(),
            )

        except tf2_py.LookupException as ex:
            # This is most likely a Transform execption, which is expected and therefore None is ok.
            # It is very likely
            return None
