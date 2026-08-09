import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from crazyflie_interfaces.srv import AddCrazyflie
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class SitlCreator(Node):
    """Create each configured SITL Crazyflie exactly once."""

    def __init__(self):
        super().__init__("sitl_creator")
        self.declare_parameter(
            "setup_yaml",
            get_package_share_directory("pad_management")
            + "/config/flies_config_sitl.yaml",
            ParameterDescriptor(read_only=True),
        )

        setup_yaml = self.get_parameter("setup_yaml").value
        with open(setup_yaml, encoding="utf-8") as config_file:
            self.flies = yaml.safe_load(config_file)["flies"]

        self.add_client = self.create_client(
            AddCrazyflie,
            "/crazyflie_hardware_gateway/add_crazyflie",
        )

    def create_all(self):
        if not self.add_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "Crazyflie hardware gateway is unavailable; no SITL Crazyflies were created."
            )
            return

        for flie in self.flies:
            self._create_once(flie)

    def _create_once(self, flie):
        cf_id = flie["id"]
        request = AddCrazyflie.Request()
        request.uri = f"radio://0/{flie['channel']}/2/E7E7E7E7{cf_id:02X}"
        (
            request.initial_pose.position.x,
            request.initial_pose.position.y,
            request.initial_pose.position.z,
        ) = flie.get("initial_position", [0.0, 0.0, 0.0])
        request.type = "default"

        future = self.add_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error(f"Failed to create SITL Crazyflie {cf_id}")
            return
        if not response.success:
            self.get_logger().error(
                f"Failed to create SITL Crazyflie {cf_id}: {response.msg}"
            )
            return

        self.get_logger().info(f"Created SITL Crazyflie {cf_id}")
        self._configure_padflie_once(cf_id)

    def _configure_padflie_once(self, cf_id):
        client = self.create_client(ChangeState, f"/padflie{cf_id}/change_state")
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f"Padflie {cf_id} is unavailable; skipping configuration."
            )
            return

        request = ChangeState.Request()
        request.transition.id = LifecycleState.TRANSITION_STATE_CONFIGURING
        request.transition.label = "configure"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().warning(f"Failed to configure Padflie {cf_id}")


def main(args=None):
    rclpy.init(args=args)
    node = SitlCreator()
    try:
        node.create_all()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
