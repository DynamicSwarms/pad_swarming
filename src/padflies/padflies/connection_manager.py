from rclpy import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from padflies_interfaces.srv import Connect
from std_srvs.srv import Empty as EmptySrv
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from padflies_interfaces.msg import AvailabilityInfo


class ConnectionManager:
    __connected = False

    def __init__(self, node: Node, prefix: str):
        callback_group = MutuallyExclusiveCallbackGroup()

        availability_rate: float = 1.0

        self._connect_service = node.create_service(
            srv_type=Connect,
            srv_name=prefix + "/connect",
            callback=self.handle_connect_request,
            callback_group=callback_group,
        )
        self._disconnect_service = node.create_service(
            srv_type=EmptySrv,
            srv_name=prefix + "/disconnect",
            callback=self.handle_disconnect_request,
            callback_group=callback_group,
        )

        # Create a QoS profile for maximum performance
        qos_profile_performance = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Minimal latency, no retries
            durability=DurabilityPolicy.VOLATILE,  # Only delivers data to currently available subscribers
            history=HistoryPolicy.KEEP_LAST,  # Keeps only the last N messages
            depth=1,  # Keeps a short history to reduce memory use
        )
        self.availability_publisher: Publisher = node.create_publisher(
            msg_type=AvailabilityInfo,
            topic="availability",
            qos_profile=qos_profile_performance,
            callback_group=callback_group,
        )

        node.create_timer(
            timer_period_sec=availability_rate,
            callback=self.send_availability_info,
            callback_group=callback_group,
        )

    def handle_connect_request(
        self, request: Connect.Request, response: Connect.Response
    ):
        if self.state == PadFlieState.IDLE and not self.__connected:
            self.__connected = True
            response.success = True
        else:
            response.success = False
        return response

    def handle_disconnect_request(
        self, req: EmptySrv.Request, response: EmptySrv.Response
    ):
        self.connected = False
        return EmptySrv.Response()

    def send_availability_info(self):
        # Publish information about us.
        # Including position , ready state etc. onto global topic.
        msg = AvailabilityInfo()
        msg.available = self.state == PadFlieState.IDLE and not self.__connected
        msg.connect_service_name = self._connect_service.srv_name
        msg.disconnect_service_name = self._disconnect_service.srv_name
        if msg.available:
            self.availability_publisher.publish(msg)
