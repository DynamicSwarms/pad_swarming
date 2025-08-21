from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.msg import NotifySetpointsStop, Position


class LowLevelCommander:
    def __init__(self, node: Node, prefix: str):
        self._node = node
        self._prefix = prefix
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._qos_profile = 10

        self.has_publishers = False

    def __del__(self):
        self.destroy_publishers()

    def create_publishers(self):
        self._notify_setpoints_stop_publisher = self._node.create_publisher(
            NotifySetpointsStop,
            self._prefix + "/notify_setpoints_stop",
            qos_profile=self._qos_profile,
            callback_group=self._callback_group,
        )

        self._position_publisher = self._node.create_publisher(
            Position,
            self._prefix + "/cmd_position",
            qos_profile=self._qos_profile,
            callback_group=self._callback_group,
        )

        self.has_publishers = True

    def destroy_publishers(self):
        if self.has_publishers:
            self.has_publishers = False

            self._node.destroy_publisher(self._notify_setpoints_stop_publisher)
            self._node.destroy_publisher(self._position_publisher)

    def notify_setpoints_stop(
        self, remain_valid_milliseconds: int = 100, group_mask: int = 0
    ) -> None:
        if not self.has_publishers:
            return
        """Send a notify setpoints command
        Informs that streaming low-level setpoint packets are about to stop.
        A common use case is to send this after the last low-level setpoint is sent but before the first high-level setpoint is sent.

        Args:
            remain_valid_milliseconds (int, optional): Artefact of pull-based hl-commander architecture no longer needed. Defaults to 100.
            group_mask (int, optional): The group this should apply to. Deprecated Dec2024. Defaults to 0.
        """
        msg = NotifySetpointsStop()
        msg.group_mask = group_mask
        msg.remain_valid_millisecs = remain_valid_milliseconds
        self._notify_setpoints_stop_publisher.publish(msg)

    def cmd_position(self, position: "list[float]", yaw: float) -> None:
        """Send a position setpoint to controller (low-level)

        Args:
            position (List[float]): Position [x, y, z] in m
            yaw (float): Orientation in degrees
        """
        if not self.has_publishers:
            return
        msg = Position()
        msg.x, msg.y, msg.z = position
        msg.yaw = yaw
        self._position_publisher.publish(msg)
