import rclpy
from rclpy.node import Node
from rclpy.task import Future

from rclpy.subscription import Subscription
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String

from typing import Optional, Callable, Tuple
import re

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State as LifecycleState, TransitionEvent

import time


class PadflieConnector:
    def __init__(
        self,
        node: Node,
        on_connect_callback: Callable[[str], None],
        on_disconnect_callback: Callable[[str], None],
    ):
        self._node = node
        self._on_connect_callback = on_connect_callback
        self._on_disconnect_callback = on_disconnect_callback

        self.__availability_subscription: Optional[Subscription] = None
        self.__transition_event_subscription: Optional[Subscription] = None
        self.__active: bool = False
        self.__connected: bool = False
        self.__padflie_prefix: str = ""

        self._connect_callback_group = MutuallyExclusiveCallbackGroup()

        self.__create_availability_subscription()

    def start_search(self):
        self.__active = True

    def stop_search(self):
        self.__active = False
        self._disconnect()

    def disconnect(self):
        self._disconnect()

    def is_active(self):
        return self.__active

    def is_connected(self) -> bool:
        return self.__connected

    def _availability_callback(self, info: String):
        if self.is_connected() or not self.is_active():
            return

        prefix = info.data
        cf_id: int = int(re.search(r"\d+", prefix).group())
        # Hardware Crazyflies have ids from A1 and onward, Webots start from 1
        if self.hardware_only and cf_id < 0xA0:
            return
        if self.webots_only and cf_id > 0xA0:
            return
        # We are active, unconnected and the cf suits us -> Connect
        self._connect(prefix)

    def _transition_event_callback(self, event: TransitionEvent):
        if event.goal_state.id == LifecycleState.TRANSITION_STATE_DEACTIVATING:
            self._node.get_logger().info(
                "Caught transition event deactivating -> Disconnected."
            )
            self.__set_disconnected()

    def _connect(self, prefix: str) -> Tuple[bool, int]:
        success = self.__transition_padflie(
            prefix=prefix,
            state=LifecycleState.TRANSITION_STATE_ACTIVATING,
            label="activate",
        )
        if success:
            self.__set_connected(prefix)
            self._node.get_logger().info(f"Connected to {prefix}")
        else:
            self._node.get_logger().info(f"Connection failed!")
        return success

    def _disconnect(self):
        # Close the connection
        if self.is_connected():
            self.__transition_padflie_async(
                prefix=self.__padflie_prefix,
                state=LifecycleState.TRANSITION_STATE_DEACTIVATING,
                label="deactivate",
            )
        # We know if disconnect worked by the transition event

    def __set_connected(self, prefix: str):
        self.__create_transition_event_subscription(prefix)
        self.__destroy_availability_subscription()
        self.__padflie_prefix = prefix
        self.__connected = True
        self._on_connect_callback(self.__padflie_prefix)

    def __set_disconnected(self):
        self.__destroy_transition_event_subscription()
        self.__create_availability_subscription()
        self.__connected = False
        self._on_disconnect_callback(self.__padflie_prefix)

    def __create_availability_subscription(self):
        # TODO: Maybe better suited qos
        self.__availability_subscription = self._node.create_subscription(
            msg_type=String,
            topic="availability",
            callback=self._availability_callback,
            qos_profile=10,
        )

    def __destroy_availability_subscription(self):
        if self.__availability_subscription is not None:
            self._node.destroy_subscription(self.__availability_subscription)
            self.__availability_subscription = None

    def __create_transition_event_subscription(self, prefix: str):
        self.__transition_event_subscription = self._node.create_subscription(
            msg_type=TransitionEvent,
            topic=f"{prefix}/transition_event",
            callback=self._transition_event_callback,
            qos_profile=10,
        )

    def __destroy_transition_event_subscription(self):
        if self.__transition_event_subscription is not None:
            self._node.destroy_subscription(self.__transition_event_subscription)
            self.__transition_event_subscription = None

    def __transition_padflie(
        self, prefix: str, state: LifecycleState, label: str
    ) -> bool:
        fut: Future = self.__transition_padflie_async(prefix, state, label)
        timeout = 1.0
        while timeout > 0.0:
            if fut.done():
                result: ChangeState.Response = fut.result()
                return result.success
            time.sleep(0.1)
            timeout -= 0.1
        self._node.get_logger().info("Connect timed out.")
        return False

    def __transition_padflie_async(
        self, prefix: str, state: LifecycleState, label: str
    ) -> Future:
        change_state_client = self._node.create_client(
            srv_type=ChangeState,
            srv_name=f"{prefix}/change_state",
            callback_group=self._connect_callback_group,
        )

        if not change_state_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(f"{prefix} is not available.")
            fut = Future()
            return fut

        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        return change_state_client.call_async(request)

    @property
    def hardware_only(self) -> bool:
        return (
            self._node.get_parameter("hardware_only").get_parameter_value().bool_value
        )

    @property
    def webots_only(self) -> bool:
        return self._node.get_parameter("webots_only").get_parameter_value().bool_value
