import rclpy
from rclpy.node import Node
from rclpy.executors import Executor
from rclpy.subscription import Subscription
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from padflies_interfaces.msg import AvailabilityInfo
from padflies_interfaces.srv import Connect
from std_srvs.srv import Empty

from typing import Optional, Callable, Tuple


class AgentConnector:
    def __init__(
        self,
        node: Node,
        executor: Executor,
        on_connect_callback: Callable[[str, int], None],
        on_disconnect_callback: Callable[[], None],
    ):
        self.__node = node
        self.__executor = executor
        self.__on_connect_callback = on_connect_callback
        self.__on_disconnect_callback = on_disconnect_callback

        self.__node.declare_parameter(name="hardware_only", value=False)

        self.__active: bool = False
        self.__connected: bool = False

        self.__disconnect_service_name: Optional[str] = None

        self.__availability_subscription: Optional[Subscription] = None
        self.__create_availability_subscription()

    def set_active(self, active: bool):
        self.__active = active
        if not active and self.__connected:
            self.disconnect()

    def is_connected(self) -> bool:
        return self.__connected

    def disconnect(self):
        self._disconnect()
        self.__create_availability_subscription()
        self.__on_disconnect_callback()

    def __availability_callback(self, info: AvailabilityInfo):
        if self.__connected or not self.__active:
            return
        if self.hardware_only and info.type != "hardware":
            return

        success, p_id = self._connect(srv_name=info.connect_service_name)
        if success:
            self.__destroy_availability_subscription()
            self.__disconnect_service_name = info.disconnect_service_name
            self.__on_connect_callback(info.padflie_prefix, p_id)

            self.__node.get_logger().info(
                f"Connected to {info.connect_service_name}, {self.__connected}"
            )
        else:
            self.__node.get_logger().info(f"Connection failed!")

    def _connect(self, srv_name: str) -> Tuple[bool, int]:
        proxy = self.__node.create_client(
            srv_type=Connect,
            srv_name=srv_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        req = Connect.Request()
        fut = proxy.call_async(req)
        rclpy.spin_until_future_complete(
            self.__node, fut, executor=self.__executor, timeout_sec=1.0
        )
        response: Optional[Connect.Response] = fut.result()
        self.__node.get_logger().info(f"{response}, {srv_name}")
        if response is not None and response.success:
            self.__connected = True
            return True, response.priority_id
        return False, 0

    def _disconnect(self):
        if self.__disconnect_service_name is not None:
            proxy = self.__node.create_client(
                srv_type=Empty,
                srv_name=self.__disconnect_service_name,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            req = Empty.Request()
            fut = proxy.call_async(req)
            rclpy.spin_until_future_complete(
                self.__node, fut, executor=self.__executor, timeout_sec=1.0
            )
            self.__connected = False
            self.__node.get_logger().info(f"Disconnected!")
        else:
            self.__node.get_logger().info(
                f"Disconnect failed. No service name available.!"
            )

    def __create_availability_subscription(self):
        # Create a QoS profile for maximum performance
        qos_profile_performance = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Minimal latency, no retries
            durability=DurabilityPolicy.VOLATILE,  # Only delivers data to currently available subscribers
            history=HistoryPolicy.KEEP_LAST,  # Keeps only the last N messages
            depth=1,  # Keeps a short history to reduce memory use
        )

        self.__availability_subscription = self.__node.create_subscription(
            msg_type=AvailabilityInfo,
            topic="availability",
            callback=self.__availability_callback,
            qos_profile=qos_profile_performance,
        )

    def __destroy_availability_subscription(self):
        if self.__availability_subscription is not None:
            self.__node.destroy_subscription(self.__availability_subscription)
            self.__availability_subscription = None

    @property
    def hardware_only(self) -> bool:
        return (
            self.__node.get_parameter("hardware_only").get_parameter_value().bool_value
        )
