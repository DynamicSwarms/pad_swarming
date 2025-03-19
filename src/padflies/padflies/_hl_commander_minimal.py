from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from crazyflie_interfaces.msg import Takeoff, Land, GoTo


class HighLevelCommander:
    def __init__(self, node: Node, prefix: str):
        self._node = node
        self._prefix = prefix
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._qos_profile = 10
        
        self.has_publishers = False

    def __del__(self):
        self.destroy_publishers()

    def create_publishers(self):
        self._takeoff_publisher = self._node.create_publisher(
            Takeoff,
            self._prefix + "/takeoff",
            qos_profile=self._qos_profile,
            callback_group=self._callback_group,
        )

        self._land_publisher = self._node.create_publisher(
            Land,
            self._prefix + "/land",
            qos_profile=self._qos_profile,
            callback_group=self._callback_group,
        )

        self._goto_publisher = self._node.create_publisher(
            GoTo, 
            self._prefix + "/go_to",
            qos_profile=self._qos_profile,
            callback_group=self._callback_group
        )

        self.has_publishers = True


    def destroy_publishers(self):
        self._node.destroy_publisher(self._takeoff_publisher)
        self._node.destroy_publisher(self._land_publisher)
        self._node.destroy_publisher(self._goto_publisher)

        self.has_publishers = False

    def takeoff(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = None,
        use_current_yaw: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Vertical takeoff from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.

        Args:
            target_height (float): height to takeoff to (absolute) in meters
            duration_seconds (float): time it should take until target_height is reached in seconds
            yaw (float): Target orientation in radians
            use_current_yaw (bool): If true use ignore yaw parameter. Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = Takeoff()
        msg.group_mask = group_mask
        msg.height = target_height
        msg.use_current_yaw = use_current_yaw
        if not yaw is None:
            msg.yaw = yaw
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self._takeoff_publisher.publish(msg)

    def land(
        self,
        target_height: float,
        duration_seconds: float,
        yaw: float = None,
        group_mask: int = 0,
    ) -> None:
        """Vertical landing from current x-y position to given height (high-level)

        The Crazyflie will hover indefinetely after target_height is reached.
        This should usually be followed by a stop command, but is not strictly required.

        Args:
            target_height (float): _description_
            duration_seconds (float): _description_
            yaw (float, optional): _description_. Defaults to None.
            group_mask (int, optional): _description_. Defaults to 0.
        """
        msg = Land()
        msg.group_mask = group_mask
        msg.height = target_height
        msg.use_current_yaw = yaw is None
        if not msg.use_current_yaw:
            msg.yaw = yaw
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self._land_publisher.publish(msg)

    def go_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        duration_seconds: float,
        relative: bool = False,
        linear: bool = False,
        group_mask: int = 0,
    ) -> None:
        """Move to x, y, z, yaw in duration_seconds amount of time (high-level)

        The Crazyflie will hover indefinetely afterwards.
        Calling goTo rapidly (> 1Hz) can cause instability. Consider using the cmd_position() setpoint command from
        generic_commander instead.

        Args:
            x (float): x-position of goal in meters
            y (float): y-position of goal in meters
            z (float): z-position of goal in meters
            yaw (float): target yaw in radians
            duration_seconds (float): Time in seconds it should take the CF to move to goal
            relative (bool, optional): If true the goal and yaw are interpreted as relative to current position. Defaults to False.
            linear (bool, optional): If true a linear interpolation is used for trajectory instead of a smooth polynomial . Defaults to False.
            group_mask (int, optional): mask for which CFs this should apply to. Defaults to 0.
        """
        msg = GoTo()
        msg.group_mask = group_mask
        msg.goal = Point(x=x, y=y, z=z)
        msg.yaw = yaw
        msg.relative = relative
        msg.linear = linear
        msg.duration = self.__seconds_to_duration(duration_seconds)
        self._goto_publisher.publish(msg)

    def __seconds_to_duration(self, seconds: float) -> Duration:
        i_seconds = int(seconds)
        fractional_seconds = seconds - i_seconds
        nanoseconds = int(fractional_seconds * 1_000_000_000)
        duration = Duration()
        duration.sec = i_seconds
        duration.nanosec = nanoseconds        
        return duration