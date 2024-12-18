import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State as LifecycleState

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Point
from pad_management_interfaces.srv import PointFinder
from ament_index_python.packages import get_package_share_directory


import yaml
import numpy as np
from itertools import cycle

from typing import List, Tuple, Dict
from dataclasses import dataclass


@dataclass
class Crazyflie:
    change_state: Client
    get_state: Client


class PadCreator(Node):
    """Manages the creation of crazyflies.

    What about removal???
    """

    def __init__(self):
        super().__init__("pad_creator")
        self.declare_parameter(
            "padflie_yaml",
            get_package_share_directory("pad_management")
            + "/config/padflie_arrangement.yaml",
        )
        self.declare_parameter("max_deviation_distance", 0.05)

        yaml_file = (
            self.get_parameter("padflie_yaml").get_parameter_value().string_value
        )
        self.max_distance = (
            self.get_parameter("max_deviation_distance")
            .get_parameter_value()
            .double_value
        )

        file = open(yaml_file, "r")
        flies = yaml.safe_load(file)["flies"]
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

        rate_hz = 20.0  # find a new crazyflie with this rate
        self.create_timer(
            timer_period_sec=1.0 / rate_hz,
            callback=self.update_crazyflies,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.crazyflies: Dict[int, Crazyflie] = {}
        self.crazyflies_callback_group = MutuallyExclusiveCallbackGroup()

    def update_crazyflies(self):
        flie = next(self.flies)
        pad_position = self._get_pad_position(flie["pad"])
        if pad_position is None:
            return

        existence = self._check_point_existance(pad_position)
        if not existence:
            return

        cf_id = flie["id"]
        if cf_id not in self.crazyflies.keys():
            # The Crazyflie exists and we need to transition its padflie into configure
            self._create_crazyflie(cf_id)
            self._transition_crazyflie(
                cf_id, LifecycleState.TRANSITION_STATE_CONFIGURING, "configure"
            )

        else:
            # If it is configured we switch to active
            # if it is in failure we remvoe
            state: LifecycleState = self._get_state(cf_id)
            if state == LifecycleState.PRIMARY_STATE_INACTIVE:
                # We dont now how we got into inactive but have to act differently
                self._transition_crazyflie(
                    cf_id, LifecycleState.TRANSITION_STATE_ACTIVATING, "activate"
                )

    def _transition_crazyflie(self, cf_id: int, state: LifecycleState, label: str):
        request = ChangeState.Request()
        request.transition.id = state
        request.transition.label = label
        if cf_id in self.crazyflies.keys():
            self.get_logger().info(str(request))
            self.crazyflies[cf_id].change_state.call_async(request)
            self.get_logger().info(f"Sent to {label}")

    def _get_state(self, cf_id: int) -> LifecycleState:
        request = GetState.Request()
        if cf_id in self.crazyflies.keys():
            response: GetState.Response = self.crazyflies[cf_id].get_state.call(request)
            return response.current_state

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

        self.crazyflies[cf_id] = Crazyflie(change_state_client, get_state_client)

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
        pass


if __name__ == "__main__":
    main()
