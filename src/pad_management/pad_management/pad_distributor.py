import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

from pad_management_interfaces.srv import AcquireCrazyflie

from ament_index_python.packages import get_package_share_directory

import yaml

from typing import List, Optional


class PadDistributor(Node):
    """A Node managing the distribution of crazyflies.

    Other nodes can ask for a crazyflie.
    They get permission to use a crazyflie, and will get notified if battery is empty etc.
    """

    def __init__(self):
        super().__init__("pad_distributor")

        self.declare_parameter(
            name="padflie_yamls",
            value=[
                get_package_share_directory("pad_management")
                + "/config/flies_config_hardware.yaml"
            ],
            descriptor=ParameterDescriptor(read_only=True),
        )

        yaml_files = (
            self.get_parameter("padflie_yamls").get_parameter_value().string_array_value
        )

        # There can be multiple yamls of flies which need to be combined.
        flies: List = []
        for yaml_file in yaml_files:
            file = open(yaml_file, "r")
            flies += yaml.safe_load(file)["flies"]

        self.create_service(
            srv_type=AcquireCrazyflie,
            srv_name="acquire_crazyflie",
            callback=self._acquire_crazyflie_callback,
        )

    def select_crazyflie(self, type: Optional[str] = None) -> Optional[int]:
        return 0

    def _acquire_crazyflie_callback(
        self, request: AcquireCrazyflie.Request, response: AcquireCrazyflie.Response
    ):
        cf_id = self.select_crazyflie(
            type=request.type if not request.allow_any_type else None
        )

        response.success = cf_id is not None
        if response.success:
            response.id = cf_id

        return response


def main():
    rclpy.init()

    pd = PadDistributor()
    executor = SingleThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(node=pd, timeout_sec=0.1, executor=executor)
        rclpy.shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
