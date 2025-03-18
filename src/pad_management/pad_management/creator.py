import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client
from rclpy.subscription import Subscription
from rcl_interfaces.msg import ParameterDescriptor

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State as LifecycleState, TransitionEvent

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Point
from pad_management_interfaces.srv import PointFinder
from ament_index_python.packages import get_package_share_directory


import yaml
import numpy as np
from itertools import cycle

from typing import List, Tuple, Dict, Set
from dataclasses import dataclass
from threading import Lock

from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie
from crazyflie_hardware_gateway_interfaces.srv import (
    AddCrazyflie as AddHardwareCrazyflie,
    RemoveCrazyflie as RemoveHardwareCrazyflie,
)

from typing import Optional, Union, Callable

from enum import Enum, auto


@dataclass
class CreationFlie:
    callback: Callable[[int, bool], None]
    cf_id: int
    cf_channel: int
    initial_position: list[float]
    type: str


class BackendType(Enum):
    HARDWARE = auto()
    WEBOTS = auto()


class Adder:
    def __init__(self, node: Node, backend: BackendType):
        self._node = node
        self.backend: BackendType = backend

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

        if not self.add_client.wait_for_service(timeout_sec=3.0):
            self._node.get_logger().info("Gateway not reachable! (ADD)")
        if not self.remove_client.wait_for_service(timeout_sec=3.0):
            self._node.get_logger().info("Gateway not reachable! (REMOVE)")

        self.add_queue: list[CreationFlie] = []
        self.add_queue_lock: Lock = Lock()
        self._node.create_timer(
            1.0, self.create, callback_group=MutuallyExclusiveCallbackGroup()
        )

    def create(self):
        flie: Optional[CreationFlie] = None
        with self.add_queue_lock:
            if len(self.add_queue):
                flie = self.add_queue.pop()
        if flie is None:
            return

        self._node.get_logger().info(f"Adding Crazyflie with ID:{flie.cf_id}")
        resp = self.add_client.call(self.create_add_request(flie))
        success = self.interpret_add_response(resp)
        flie.callback(flie.cf_id, success)

    def interpret_add_response(
        self, response: Union[AddHardwareCrazyflie.Response | WebotsCrazyflie.Response]
    ) -> bool:
        if self.backend == BackendType.HARDWARE:
            return response.success
        if self.backend == BackendType.WEBOTS:
            return response.success
        return False

    def interpret_remove_response(
        self,
        response: Union[RemoveHardwareCrazyflie.Response | WebotsCrazyflie.Response],
    ) -> bool:
        if self.backend == BackendType.HARDWARE:
            return response.success
        if self.backend == BackendType.WEBOTS:
            return response.success
        return False

    def create_add_request(
        self, flie: CreationFlie
    ) -> Union[AddHardwareCrazyflie.Request | WebotsCrazyflie.Request]:
        if self.backend == BackendType.HARDWARE:
            req = AddHardwareCrazyflie.Request()
            req.id = flie.cf_id
            req.channel = flie.cf_channel
            req.initial_position.x, req.initial_position.y, req.initial_position.z = (
                flie.initial_position
            )
            req.type = flie.type
            return req
        if self.backend == BackendType.WEBOTS:
            req = WebotsCrazyflie.Request()
            req.id = flie.cf_id
            return req

    def create_remove_request(
        self, flie: CreationFlie
    ) -> Union[RemoveHardwareCrazyflie.Request | WebotsCrazyflie.Request]:
        if self.backend == BackendType.HARDWARE:
            req = RemoveHardwareCrazyflie.Request()
            req.id = flie.cf_id
            req.channel = flie.cf_channel
            return req
        if self.backend == BackendType.WEBOTS:
            req = WebotsCrazyflie.Request()
            req.id = flie.cf_id
            return req

    def enqueue_creation(
        self,
        callback: Callable[[int, bool], None],
        cf_id: int,
        cf_channel: int = 80,
        initial_position: list[float] = [0.0, 0.0, 0.0],
        type: str = "default",
    ):
        with self.add_queue_lock:
            self.add_queue.append(
                CreationFlie(
                    callback=callback,
                    cf_id=cf_id,
                    cf_channel=cf_channel,
                    initial_position=initial_position,
                    type=type,
                )
            )


@dataclass
class Crazyflie:
    change_state: Client
    get_state: Client
    transition_event: Subscription
    cooldown: float = (
        None  # If this is set the crazyflie is said dead but will be removed from our list only if this cooldown is passed
    )


class Creator(Node):
    """Manages the creation of crazyflies."""

    def __init__(self):
        super().__init__("creator")
        self.crazyflies_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # Listening to transition Events

        # Declare parameters
        self.declare_parameter(
            name="setup_yaml",
            value=get_package_share_directory("pad_management")
            + "/config/lighthouse_config.yaml",
            descriptor=ParameterDescriptor(read_only=True),
        )
        # Read parameters
        yaml_file = self.get_parameter("setup_yaml").get_parameter_value().string_value

        # Load flies from yaml
        flies: List = []
        file = open(yaml_file, "r")
        flies += yaml.safe_load(file)["flies"]

        # Randomly permutate to add them in new order every time the system gets restarted.
        flies = np.random.permutation(flies)
        self.flies = cycle(flies)

        self.adder = Adder(self, BackendType.HARDWARE)
        self.added: list[int] = []

        self.create_timer(
            timer_period_sec=1.0,
            callback=self.update_crazyflies,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def update_crazyflies(self):
        flie = next(self.flies)

        cf_id = flie["id"]
        if cf_id in self.added:
            return  # We already try to create.

        # Add the crazyflie and transition it to Configuring
        self.adder.enqueue_creation(self.on_add_callback, cf_id, flie["channel"])
        self.added.append(cf_id)

    def on_add_callback(self, cf_id: int, success: bool):
        if success:
            self.create_subscription(
                msg_type=TransitionEvent,
                topic=f"cf{cf_id}/transition_event",
                callback=lambda event: self._transition_event_callback(cf_id, event),
                qos_profile=10,
            )

            self._transition_padflie(
                cf_id=cf_id,
                state=LifecycleState.TRANSITION_STATE_CONFIGURING,
                label="configure",
            )
        else:
            self.added.remove(cf_id)

    def _transition_padflie(self, cf_id: int, state: LifecycleState, label: str):
        change_state_client = self.create_client(
            srv_type=ChangeState,
            srv_name=f"padflie{cf_id}/change_state",
            callback_group=self.crazyflies_callback_group,
        )
        if not change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"The Padflie with ID {cf_id} is not available.")
            return  ## This permanently disables this crazyflie.

        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        change_state_client.call_async(request)

    def _transition_event_callback(self, cf_id: int, event: TransitionEvent):
        if cf_id in self.added:
            if event.goal_state.id == LifecycleState.TRANSITION_STATE_SHUTTINGDOWN:
                self.get_logger().info(f"Caught {cf_id} failure. Trying to reconnect.")
                self.added.remove(cf_id)  # Retry to connect


def main():
    rclpy.init()

    pc = Creator()
    executor = (
        MultiThreadedExecutor()
    )  # Because we are calling a service inside a timer
    try:
        while rclpy.ok():
            rclpy.spin_once(node=pc, timeout_sec=0.1, executor=executor)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
