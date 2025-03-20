from crazyflies.crazyflie import CrazyflieType

from rclpy.node import Node
from crazyflie_interfaces.msg import GenericLogData

from typing import Callable

class ChargeController:

    def __init__(
        self, node: Node, cf_type: CrazyflieType, cf_prefix: str, on_charged_callback: Callable[[],None]
    ):
        self._node = node
        self._cf_type = cf_type
        self._on_charged_callback = on_charged_callback

        if cf_type == CrazyflieType.HARDWARE:
            self._battery_subscription = self._node.create_subscription(msg_type=GenericLogData, topic=cf_prefix + "/state", callback=self.receive_data, qos_profile=10 )
            self.voltage = 0.0
        else:
            self._fully_charged_timer = self._node.create_timer(1.0, self._webots_pub_charged_callback)
            self.voltage = 4.2  # A hardware crazyflie is always charged.

    def destroy_subscriptions(self): 
        if self._cf_type == CrazyflieType.HARDWARE: 
            self._node.destroy_subscription(self._battery_subscription)
        if self._cf_type == CrazyflieType.WEBOTS: 
            self._node.destroy_timer(self._fully_charged_timer)

    def is_charged(self):
        return self.voltage > self.battery_voltage_charged

    def is_empty(self):
        return self.voltage < self.battery_voltage_empty

    def get_voltage(self):
        return self.voltage

    def receive_data(self, msg:GenericLogData):
        variables = msg.values
        if not len(variables):
            return
        
        # Variables: "pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled", "sys.armed"
        self.voltage = variables[0]

        if self.is_empty():
            self._node.get_logger().info("Battery critical: " + str(self.voltage))
        if self.is_charged(): 
            self._on_charged_callback()

    def _webots_pub_charged_callback(self):
        self._on_charged_callback()

    @property
    def battery_voltage_empty(self):
        return self._node.get_parameter("battery_voltage_empty").get_parameter_value().double_value
    @property
    def battery_voltage_charged(self):
        return self._node.get_parameter("battery_voltage_charged").get_parameter_value().double_value