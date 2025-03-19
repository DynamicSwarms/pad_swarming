from rclpy.node import Node

from ._padflie_tf import PadflieTF
from .connection_manager import ConnectionManager
from .charge_controller import ChargeController
from .commander import PadflieCommander

from typing import Callable
from crazyflies.crazyflie import CrazyflieType

from std_msgs.msg import String

class PadflieController:
    def __init__(
        self,
        node: Node,
        cf_type: CrazyflieType,
        prefix: str,
        cf_prefix: str,
        tf_manager: PadflieTF,
        sleep: Callable[[float], None],
    ):
        self._node = node
        self._cf_type = cf_type
        self._prefix = prefix
        self._cf_prefix = cf_prefix
        self._sleep = sleep

        self._world = "world"

        self._commander = PadflieCommander(
            node=node,
            prefix=prefix,
            cf_prefix=cf_prefix,
            tf_manager=tf_manager,
            sleep=sleep,
        )

        # For now the charge controller is the only one who is always active
        self._charge_controller = ChargeController(
            node=node, cf_type=cf_type, cf_prefix=cf_prefix, on_charged_callback=self.on_battery_full_callback
        )  # Checks charge state

        self.availability_publisher = self._node.create_publisher(String, "availability", qos_profile=10)
        self.activated = False

    def on_battery_full_callback(self):
        if not self.activated: 
            msg = String()
            msg.data = self._prefix
            self.availability_publisher.publish(msg) # Say "padflieID" if available.

    def activate(self) -> bool:
        if not self._charge_controller.is_charged(): 
            return False
        
        self._commander.activate()
        self.activated = True
        return True
    
    def deactivate(self):
        self._commander.deactivate()
        self.activated = False