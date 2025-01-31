from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher
from padflies_interfaces.srv import Connect
from std_srvs.srv import Empty as EmptySrv
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from padflies_interfaces.msg import AvailabilityInfo

from ._padflie_states import PadFlieState
from ._qos_profiles import qos_profile_performance


class ConnectionManager:

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()

        self.__connected = False
        self.__availability_rate: float = 1.0

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

    def _handle_connect_request(
        self, request: Connect.Request, response: Connect.Response
    ):
        if self.__connected:  # Check padflie state IDLE ??
            self.__connected = True
            response.success = True
        else:
            response.success = False
        return response

    def _handle_disconnect_request(
        self, req: EmptySrv.Request, response: EmptySrv.Response
    ):
        self.connected = False
        return EmptySrv.Response()

    def _send_availability_info(self):
        # Publish information about us.
        # Including position , ready state etc. onto global topic.
        msg = AvailabilityInfo()
        msg.available = (
            self.__connected
        )  # check also: self.state == PadFlieState.IDLE and not self.__connected
        msg.connect_service_name = self._connect_service.srv_name
        msg.disconnect_service_name = self._disconnect_service.srv_name
        if msg.available:
            self.availability_publisher.publish(msg)
