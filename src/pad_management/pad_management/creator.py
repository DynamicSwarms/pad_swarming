from rclpy.node import Node
import rclpy.time
from rclpy.subscription import Subscription
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.client import Client

from dataclasses import dataclass
from threading import Lock

from crazyflie_interfaces.srv import AddCrazyflie, RemoveCrazyflie

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


class Creator:
    def __init__(
        self,
        node: Node,
        backend: BackendType,
        on_add_callback: Callable[[int, bool, str], None],
        on_failure_callback: Callable[[int], None],
        max_concurrent_adds: int = 10,
    ):
        if max_concurrent_adds < 1:
            raise ValueError("max_concurrent_adds must be at least 1")
        self.max_concurrent_adds = max_concurrent_adds
        self._pending_adds: set[int] = set()
        self._node = node
        self.backend: BackendType = backend
        self.on_add_callback = on_add_callback
        self.on_failure_callback = on_failure_callback

        if self.backend == BackendType.HARDWARE:
            self.add_client: Client = self._node.create_client(
                AddCrazyflie,
                "crazyflie_hardware_gateway/add_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.remove_client: Client = self._node.create_client(
                RemoveCrazyflie,
                "crazyflie_hardware_gateway/remove_crazyflie",
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        #if self.backend == BackendType.WEBOTS:
        #    self.add_client: Client = self._node.create_client(
        #        WebotsCrazyflie,
        #        "crazyflie_webots_gateway/add_crazyflie",
        #        callback_group=MutuallyExclusiveCallbackGroup(),
        #    )
        #    self.remove_client: Client = self._node.create_client(
        #        WebotsCrazyflie,
        #        "crazyflie_webots_gateway/remove_crazyflie",
        #        callback_group=MutuallyExclusiveCallbackGroup(),
        #    )

        self._subscription_lock = Lock()
        self._subscription_attempts: Dict[int, object] = {}
        self._transition_event_subscriptions: Dict[int, Subscription] = {}
        self._transition_event_callback_group = MutuallyExclusiveCallbackGroup()

        self.add_queue: "list[CreationFlie]" = []
        self.add_queue_lock: Lock = Lock()
        self._creation_callback_group = ReentrantCallbackGroup()
        self._node.create_timer(
            0.1,
            self.create_timer_callback,
            callback_group=self._creation_callback_group,
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
        with self.add_queue_lock:
            if not self.add_queue or len(self._pending_adds) >= self.max_concurrent_adds:
                return
            if not self.add_client.service_is_ready():
                return
            flie = self.add_queue.pop()
            self._pending_adds.add(flie.cf_id)

        attempt = object()
        try:
            with self._subscription_lock:
                old = self._transition_event_subscriptions.pop(flie.cf_id, None)
                self._subscription_attempts.pop(flie.cf_id, None)
                if old is not None:
                    self._node.destroy_subscription(old)
                self._transition_event_subscriptions[flie.cf_id] = self._node.create_subscription(
                    msg_type=TransitionEvent,
                    topic=f"cf{flie.cf_id}/transition_event",
                    callback=lambda event: self._transition_event_callback(
                        flie.cf_id, event, attempt
                    ),
                    qos_profile=10,
                    callback_group=self._transition_event_callback_group,
                )
                self._subscription_attempts[flie.cf_id] = attempt
            future = self.add_client.call_async(self._create_add_request(flie))
        except Exception as exc:
            self._finish_add(flie.cf_id, attempt, False, str(exc))
            return

        def on_response(future):
            try:
                success, msg = self._interpret_add_response(future.result())
            except Exception as exc:
                success, msg = False, str(exc)
            self._finish_add(flie.cf_id, attempt, success, msg)

        future.add_done_callback(on_response)

    def _finish_add(self, cf_id: int, attempt: object, success: bool, msg: str):
        try:
            if not success:
                self._remove_subscription(cf_id, attempt)
            self.on_add_callback(cf_id, success, msg)
        finally:
            with self.add_queue_lock:
                self._pending_adds.discard(cf_id)

    def _remove_subscription(self, cf_id: int, attempt: object) -> bool:
        with self._subscription_lock:
            if self._subscription_attempts.get(cf_id) is not attempt:
                return False
            self._subscription_attempts.pop(cf_id)
            subscription = self._transition_event_subscriptions.pop(cf_id)
            self._node.destroy_subscription(subscription)
            return True

    def _transition_event_callback(
        self, cf_id: int, event: TransitionEvent, attempt: object
    ):
        if event.goal_state.id == LifecycleState.TRANSITION_STATE_SHUTTINGDOWN:
            if self._remove_subscription(cf_id, attempt):
                self.on_failure_callback(cf_id)

    def _interpret_add_response(
        self,
        response: AddCrazyflie.Response,
    ) -> "tuple[bool, str]":
        return response.success, response.msg
    
    def _interpret_remove_response(
        self,
        response: RemoveCrazyflie.Response,
    ) -> "tuple[bool, str]":
        if self.backend == BackendType.HARDWARE:
            return response.success, response.msg
        if self.backend == BackendType.WEBOTS:
            return response.success, ""
        return False, ""

    def _create_add_request(
        self, flie: CreationFlie
        ) -> AddCrazyflie.Request:
        if self.backend == BackendType.HARDWARE:
            request = AddCrazyflie.Request()
            request.uri = f"radio://0/{flie.cf_channel}/2/E7E7E7E7{flie.cf_id:02X}"
            (
                request.initial_pose.position.x,
                request.initial_pose.position.y,
                request.initial_pose.position.z,
            ) = flie.initial_position
            request.type = flie.type
            return request

       
    def _create_remove_request(
        self, flie: CreationFlie
    ) -> RemoveCrazyflie.Request:
        request = RemoveCrazyflie.Request()
        request.uri = f"radio://0/{flie.cf_channel}/2/E7E7E7E7{flie.cf_id:02X}"
        return request
