from crazyflie_interfaces_python.client import LoggingClient, LogBlockClient
from crazyflies.crazyflie import CrazyflieType

from rclpy.node import Node


class ChargeController:

    def __init__(
        self, node: Node, cf_type: CrazyflieType, logging_client: LoggingClient
    ):
        self.node = node
        if cf_type == CrazyflieType.HARDWARE:
            block = logging_client.create_log_block(
                ["pm.vbat"], "pm", self.receive_data
            )
            block.start_log_block(100)

            self.voltage = 0.0
        else:
            self.voltage = 4.2  # A hardware crazyflie is always charged.

    def is_charged(self):
        return self.voltage > 4.12

    def is_empty(self):
        return self.voltage < 3.2

    def get_voltage(self):
        return self.voltage

    def receive_data(self, variables):
        if not len(variables):
            return
        self.voltage = variables[0]

        if self.is_empty():
            self.node.get_logger().info("Battery critical: " + str(self.voltage))
