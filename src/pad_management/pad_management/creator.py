from rclpy.node import Node
import rclpy.time
from rclpy.subscription import Subscription
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.client import Client

from dataclasses import dataclass
from threading import Lock

from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie
from crazyflie_hardware_gateway_interfaces.srv import (
    AddCrazyflie as AddHardwareCrazyflie,
    RemoveCrazyflie as RemoveHardwareCrazyflie,
)

from lifecycle_msgs.msg import State as LifecycleState, TransitionEvent


from typing import Optional, Union, Callable, Dict

from enum import Enum, auto


@dataclass
class CreationFlie:
    cf_id: int
    cf_channel: int
    initial_position: "list[float]"
    type: str


class BackendType(Enum):
    HARDWARE = auto()
    WEBOTS = auto()


class Creator:
    def __init__(
        self,
        node: Node,
        backend: BackendType,
        on_add_callback: Callable[[int, bool, str], None],
        on_failure_callback: Callable[[int], None],
    ):
        self._node = node
        self.backend: BackendType = backend
        self.on_add_callback = on_add_callback
        self.on_failure_callback = on_failure_callback

        if self.backend == BackendType.HARDWARE:
            self.add_client: Client = self._node.create_client(
                AddHardwareCrazyflie,
                "crazyflie_hardware_gateway/add_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
                # We might want to make this reentrant in the future,
                # but at the moment the gateway isnt parallel as well
            )
            self.remove_client: Client = self._node.create_client(
                RemoveHardwareCrazyflie,
                "crazyflie_hardware_gateway/remove_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        if self.backend == BackendType.WEBOTS:
            self.add_client: Client = self._node.create_client(
                WebotsCrazyflie,
                "crazyflie_webots_gateway/add_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.remove_client: Client = self._node.create_client(
                WebotsCrazyflie,
                "crazyflie_webots_gateway/remove_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        if not self.add_client.wait_for_service(timeout_sec=0.5):
            self._node.get_logger().info("Gateway not reachable! (ADD)")
        if not self.remove_client.wait_for_service(timeout_sec=0.5):
            self._node.get_logger().info("Gateway not reachable! (REMOVE)")

        self._transition_event_subscriptions: Dict[int, Subscription] = {}
        self._transition_event_callback_group = MutuallyExclusiveCallbackGroup()

        self.add_queue: "list[CreationFlie]" = []
        self.add_queue_lock: Lock = Lock()
        self._node.create_timer(
            0.1,
            self.create_timer_callback,
            callback_group=self._transition_event_callback_group,
        )

    def enqueue_creation(
        self,
        cf_id: int,
        cf_channel: int = 80,
        initial_position: "list[float]" = [0.0, 0.0, 0.0],
        type: str = "default",
    ):
        with self.add_queue_lock:
            self.add_queue.append(
                CreationFlie(
                    cf_id=cf_id,
                    cf_channel=cf_channel,
                    initial_position=initial_position,
                    type=type,
                )
            )

    def create_timer_callback(self):
        flie: Optional[CreationFlie] = None
        with self.add_queue_lock:
            if len(self.add_queue):
                flie = self.add_queue.pop()
        if flie is None:
            return

        self._node.get_logger().debug(f"Creator adding Crazyflie with ID:{flie.cf_id}")
        self._transition_event_subscriptions[flie.cf_id] = (
            self._node.create_subscription(
                msg_type=TransitionEvent,
                topic=f"cf{flie.cf_id}/transition_event",
                callback=lambda event: self._transition_event_callback(
                    flie.cf_id, event
                ),
                qos_profile=10,
                callback_group=self._transition_event_callback_group,
            )
        )
        resp = self.add_client.call(self._create_add_request(flie))
        success, msg = self._interpret_add_response(resp)

        if not success:
            self._node.destroy_subscription(
                self._transition_event_subscriptions[flie.cf_id]
            )

        self.on_add_callback(flie.cf_id, success, msg)

    def _transition_event_callback(self, cf_id: int, event: TransitionEvent):
        if event.goal_state.id == LifecycleState.TRANSITION_STATE_SHUTTINGDOWN:
            if cf_id in self._transition_event_subscriptions.keys():
                self._node.destroy_subscription(
                    self._transition_event_subscriptions[cf_id]
                )

            self.on_failure_callback(cf_id)

    def _interpret_add_response(
        self,
        response: "Union[AddHardwareCrazyflie.Response | WebotsCrazyflie.Response]",
    ) -> "tuple[bool, str]":
        if self.backend == BackendType.HARDWARE:
            return response.success, response.msg
        if self.backend == BackendType.WEBOTS:
            return response.success, ""
        return False

    def _interpret_remove_response(
        self,
        response: "Union[RemoveHardwareCrazyflie.Response | WebotsCrazyflie.Response]",
    ) -> "tuple[bool, str]":
        if self.backend == BackendType.HARDWARE:
            return response.success, response.msg
        if self.backend == BackendType.WEBOTS:
            return response.success, ""
        return False, ""

    def _create_add_request(
        self, flie: CreationFlie
    ) -> "Union[AddHardwareCrazyflie.Request | WebotsCrazyflie.Request]":
        if self.backend == BackendType.HARDWARE:
            req = AddHardwareCrazyflie.Request()
            req.id = flie.cf_id
            req.channel = flie.cf_channel
            (
                req.initial_position.x,
                req.initial_position.y,
                req.initial_position.z,
            ) = flie.initial_position
            req.type = flie.type
            return req
        if self.backend == BackendType.WEBOTS:
            req = WebotsCrazyflie.Request()
            req.id = flie.cf_id
            return req

    def _create_remove_request(
        self, flie: CreationFlie
    ) -> "Union[RemoveHardwareCrazyflie.Request | WebotsCrazyflie.Request]":
        if self.backend == BackendType.HARDWARE:
            req = RemoveHardwareCrazyflie.Request()
            req.id = flie.cf_id
            req.channel = flie.cf_channel
            return req
        if self.backend == BackendType.WEBOTS:
            req = WebotsCrazyflie.Request()
            req.id = flie.cf_id
            return req
