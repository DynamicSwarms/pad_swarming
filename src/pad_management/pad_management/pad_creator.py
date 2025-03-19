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

from typing import Optional, Dict

from .creator import Creator, BackendType 

class PadCreator(Node):
    """Manages the creation of crazyflies."""

    def __init__(self):
        super().__init__("pad_creator")
        self.crazyflies_callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # Listening to transition Events

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
        flies: list[Dict] = []
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

        self.hardware_creator = Creator(self, BackendType.HARDWARE, self.on_add_callback, self.on_failure_callback)
        self.webots_creator = Creator(self, BackendType.WEBOTS, self.on_add_callback, self.on_failure_callback)
        
        self.added: list[int] = []

        self.create_timer(
            timer_period_sec=0.1,
            callback=self.update_crazyflies,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def update_crazyflies(self):
        flie = next(self.flies)

        cf_id = flie["id"]
        if cf_id in self.added:
            return  # We already try to create.

        pad_position = self._get_pad_position(flie["pad"])
        if pad_position is None:
            return  # Was not able to find associated pad

        existence = self._check_point_existance(pad_position)
        if not existence:
            return  # Was not able to detect a Marker at pad position

        # Add the crazyflie and transition it to Configuring
        if "channel" in flie.keys():
            self.hardware_creator.enqueue_creation(cf_id=cf_id,cf_channel=flie["channel"], initial_position=pad_position,type="tracked")
        else:
            self.webots_creator.enqueue_creation(cf_id=cf_id)

        self.added.append(cf_id)
    
    def on_add_callback(self, cf_id: int, success: bool):
        self.get_logger().info(f"AddCallback:{cf_id}, {success}")
        if success:
            self._transition_padflie(
                cf_id=cf_id,
                state=LifecycleState.TRANSITION_STATE_CONFIGURING,
                label="configure",
            )
        else:
            self.added.remove(cf_id) # Retrry creation
    
    def on_failure_callback(self, cf_id: int):
        self.get_logger().info(f"FailureCallback: {cf_id}")
        if cf_id in self.added:
            self._transition_padflie(cf_id=cf_id, state=LifecycleState.TRANSITION_STATE_DEACTIVATING, label="deactivate") # Trie this
            self._transition_padflie(cf_id=cf_id, state=LifecycleState.TRANSITION_STATE_CLEANINGUP, label="cleanup") # But especially return to unconfigured
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

    def _check_point_existance(self, point: "list[float]") -> bool:
        request = PointFinder.Request()
        request.point = Point(x=point[0], y=point[1], z=point[2])
        request.max_distance = self.max_distance
        response: PointFinder.Response = self.check_for_point_client.call(request)
        return response.found

    def _get_pad_position(self, pad_id: int) -> Optional["list[float]"]:
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

    creator = PadCreator()
    executor = (
        MultiThreadedExecutor()
    )  # Because we are calling a service inside a timer
    try:
        while rclpy.ok():
            rclpy.spin_once(node=creator, timeout_sec=0.1, executor=executor)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
