from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from padflies_interfaces.srv import Connect
from std_srvs.srv import Empty as EmptySrv
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from crazyflies.crazyflie import CrazyflieType

from padflies_interfaces.msg import AvailabilityInfo, PadflieInfo

from .charge_controller import ChargeController
from .commander import PadflieCommander
from ._padflie_states import PadFlieState
from ._qos_profiles import qos_profile_performance

from typing import Optional


class ConnectionManager:

    def __init__(
        self,
        node: Node,
        cf_type: CrazyflieType,
        prefix: str,
        charge_controller: ChargeController,
        commander: PadflieCommander,
    ):
        self.__node = node
        self.__cf_type = cf_type
        self.__prefix = prefix
        self.__charge_controller = charge_controller
        self.__padflie_commander = commander

        callback_group = MutuallyExclusiveCallbackGroup()

        self.__connected = False
        self.__availability_rate: float = 1.0
        self.__info_rate: float = 0.1
        self.__info_timer: Optional[Timer] = None
        self.__info_publisher: Optional[Publisher] = None

        self.current_priority_id: int = 0

        self._connect_service = node.create_service(
            srv_type=Connect,
            srv_name=prefix + "/connect",
            callback=self._handle_connect_request,
            callback_group=callback_group,
        )
        self._disconnect_service = node.create_service(
            srv_type=EmptySrv,
            srv_name=prefix + "/disconnect",
            callback=self._handle_disconnect_request,
            callback_group=callback_group,
        )

        self.availability_publisher: Publisher = node.create_publisher(
            msg_type=AvailabilityInfo,
            topic="availability",
            qos_profile=qos_profile_performance,
            callback_group=callback_group,
        )

        node.create_timer(
            timer_period_sec=self.__availability_rate,
            callback=self._send_availability_info,
            callback_group=callback_group,
        )

    def _on_connect(self):
        self.__connected = True
        self.__padflie_commander.set_current_priority_id(self.current_priority_id)

        self.__info_timer = self.__node.create_timer(self.__info_rate, self._send_info)
        self.__info_publisher = self.__node.create_publisher(
            msg_type=PadflieInfo, topic=self.__prefix + "/info", qos_profile=10
        )

    def _on_disconnect(self):
        self.__connected = False
        self.current_priority_id += 1

        if self.__info_timer is not None:
            self.__node.destroy_timer(self.__info_timer)
            self.__info_timer = None
        if self.__info_publisher is not None:
            self.__node.destroy_timer(self.__info_publisher)
            self.__info_publisher = None

    def _handle_connect_request(
        self, request: Connect.Request, response: Connect.Response
    ):
        self.__node.get_logger().info(f"Request. {self.__connected}")
        if not self.__connected:  # Check padflie state IDLE ??
            self._on_connect()
            response.priority_id = self.current_priority_id
            response.success = True
        else:
            response.success = False
        return response

    def _handle_disconnect_request(
        self, req: EmptySrv.Request, response: EmptySrv.Response
    ):
        self._on_disconnect()
        return response

    def _send_info(self):
        msg = PadflieInfo()
        pose_world = self.__padflie_commander.get_cf_pose_world()
        if pose_world is not None:
            msg.pose_world = pose_world
            msg.pose_world_valid = True

        pose = self.__padflie_commander.get_cf_pose_stamped()
        if pose is not None:
            msg.pose = pose
            msg.pose_valid = True

        if self.__info_publisher is not None:
            self.__info_publisher.publish(msg)

    def _send_availability_info(self):
        # Publish information about us.
        # Including position, ready state etc. onto global topic.
        msg = AvailabilityInfo()
        msg.connect_service_name = self._connect_service.srv_name
        msg.disconnect_service_name = self._disconnect_service.srv_name
        msg.padflie_prefix = self.__prefix
        msg.available = (
            not self.__connected and self.__charge_controller.is_charged()
        )  # some state?

        pose_world = self.__padflie_commander.get_cf_pose_world()
        if pose_world is not None:
            msg.pose_world = pose_world

        msg.battery_voltage = self.__charge_controller.get_voltage()

        if self.__cf_type == CrazyflieType.HARDWARE:
            msg.type = "hardware"
        else:
            msg.type = "webots"

        if msg.available:
            self.availability_publisher.publish(msg)
