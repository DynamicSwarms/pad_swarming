from crazyflie_interfaces_python.client import LoggingClient, LogBlockClient

from rclpy.node import Node


class ChargeController:

    def __init__(self, node: Node, logging_client: LoggingClient):
        self.node = node
        block = logging_client.create_log_block(["pm.vbat"], "pm", self.receive_data)
        block.start_log_block(100)

        self.voltage = 0.0

    def is_charged(self):
        return self.voltage > 4.12

    def is_empty(self):
        return self.voltage < 3.2

    def receive_data(self, variables):
        if not len(variables):
            return
        self.voltage = variables[0]

        if self.is_empty():
            self.node.get_logger().info("Battery critical: " + str(self.voltage))
