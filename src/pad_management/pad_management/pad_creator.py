import rclpy
from rclpy.node import Node
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


@dataclass
class Crazyflie:
    change_state: Client
    get_state: Client
    transition_event: Subscription
    cooldown: float = (
        None  # If this is set the crazyflie is said dead but will be removed from our list only if this cooldown is passed
    )


class PadCreator(Node):
    """Manages the creation of crazyflies.

    What about removal???
    """

    def __init__(self):
        super().__init__("pad_creator")
        # Declare parameters
        self.declare_parameter(
            name="padflie_yamls",
            value=[
                get_package_share_directory("pad_management")
                + "/config/flies_config_hardware.yaml"
            ],
            descriptor=ParameterDescriptor(read_only=True),
        )
        self.declare_parameter(name="max_deviation_distance", value=0.05)

        # Read parameters
        yaml_files = (
            self.get_parameter("padflie_yamls").get_parameter_value().string_array_value
        )
        self.max_distance = (
            self.get_parameter("max_deviation_distance")
            .get_parameter_value()
            .double_value
        )

        # There can be multiple yamls of flies which need to be combined.
        flies: List = []
        for yaml_file in yaml_files:
            file = open(yaml_file, "r")
            flies += yaml.safe_load(file)["flies"]

        flies = np.random.permutation(flies)
        # Randomly permutate to add them in new order every time the system gets restarted.
        self.flies = cycle(flies)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.check_for_point_client = self.create_client(
            srv_type=PointFinder,
            srv_name="checkForPoint",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        while not self.check_for_point_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                "Pad Creator waiting for checkForPoint service. Cannot launch until available."
            )

        rate_hz = 20.0  # find a new crazyflie with this rate. Also used for cooldown before retry.
        self.dt = 1.0 / rate_hz
        self.create_timer(
            timer_period_sec=self.dt,
            callback=self.update_crazyflies,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.crazyflies: Dict[int, Crazyflie] = {}
        self.crazyflies_callback_group = MutuallyExclusiveCallbackGroup()

        self.retry_cooldown = 3.0  # Seconds before we try again if failed

    def update_crazyflies(self):
        self._cooldown_and_remove()
        flie = next(self.flies)

        cf_id = flie["id"]
        if cf_id in self.crazyflies.keys():
            # The crazyflie is already spawned.
            return

        pad_position = self._get_pad_position(flie["pad"])
        if pad_position is None:
            # Was not able to find associated pad
            return

        existence = self._check_point_existance(pad_position)
        if not existence:
            # Was not able to detect a Marker at pad position
            return

        # Add the crazyflie and transition it to Configuring
        self.get_logger().info(f"Adding Crazyflie with ID:{cf_id}")
        self._create_crazyflie(cf_id)
        self._transition_crazyflie(
            cf_id=cf_id,
            state=LifecycleState.TRANSITION_STATE_CONFIGURING,
            label="configure",
        )

    def _cooldown_and_remove(self):
        for cf_id in list(self.crazyflies.keys()):
            if self.crazyflies[cf_id].cooldown is not None:
                self.crazyflies[cf_id].cooldown -= self.dt
                if self.crazyflies[cf_id].cooldown < 0.0:
                    del self.crazyflies[cf_id]

    def _create_crazyflie(self, cf_id: int):
        change_state_client = self.create_client(
            srv_type=ChangeState,
            srv_name=f"padflie{cf_id}/change_state",
            callback_group=self.crazyflies_callback_group,
        )
        get_state_client = self.create_client(
            srv_type=GetState,
            srv_name=f"padflie{cf_id}/get_state",
            callback_group=self.crazyflies_callback_group,
        )
        transition_event_subscription = self.create_subscription(
            msg_type=TransitionEvent,
            topic=f"padflie{cf_id}/transition_event",
            callback=lambda event: self._transition_event_callback(cf_id, event),
            qos_profile=10,
        )

        self.crazyflies[cf_id] = Crazyflie(
            change_state_client, get_state_client, transition_event_subscription
        )

    def _transition_crazyflie(self, cf_id: int, state: LifecycleState, label: str):
        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        if cf_id in self.crazyflies.keys():
            self.crazyflies[cf_id].change_state.call_async(request)

    def _get_state(self, cf_id: int) -> LifecycleState:
        request = GetState.Request()
        if cf_id in self.crazyflies.keys():
            response: GetState.Response = self.crazyflies[cf_id].get_state.call(request)
            return response.current_state

    def _transition_event_callback(self, cf_id: int, event: TransitionEvent):
        if cf_id in self.crazyflies.keys():
            if event.goal_state.id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
                self.crazyflies[cf_id].cooldown = self.retry_cooldown
            if event.goal_state.id == LifecycleState.TRANSITION_STATE_SHUTTINGDOWN:
                pass  # Crazyflie shall live

    def _check_point_existance(self, point: List[float]) -> bool:
        request = PointFinder.Request()
        request.point = Point(x=point[0], y=point[1], z=point[2])
        request.max_distance = self.max_distance
        response: PointFinder.Response = self.check_for_point_client.call(request)
        return response.found

    def _get_pad_position(self, pad_id: int) -> List[float]:
        pad_name = f"pad_{pad_id}"
        try:
            t = self.tf_buffer.lookup_transform("world", pad_name, rclpy.time.Time())
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception as ex:
            self.get_logger().debug(str(ex))
            return None


def main():
    rclpy.init()

    pc = PadCreator()
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
