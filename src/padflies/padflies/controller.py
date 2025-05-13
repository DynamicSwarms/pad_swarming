from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ._padflie_tf import PadflieTF
from .connection_manager import ConnectionManager
from .charge_controller import ChargeController
from .commander import PadflieCommander

from typing import Callable, List, Optional
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
        clipping_box: Optional[List[float]]
    ):
        self._node = node
        self._cf_type = cf_type
        self._prefix = prefix
        self._cf_prefix = cf_prefix
        self._sleep = sleep
        self._clipping_box = clipping_box

        self._world = "world"

        # For now the charge controller is the only one who is always active
        self._charge_controller = ChargeController(
            node=node,
            cf_type=cf_type,
            cf_prefix=cf_prefix,
            on_charged_callback=self.on_battery_full_callback,
        )  # Checks charge state

        self._commander = PadflieCommander(
            node=node,
            prefix=prefix,
            cf_prefix=cf_prefix,
            tf_manager=tf_manager,
            charge_controller=self._charge_controller,
            sleep=sleep,
            clipping_box=self._clipping_box
        )

        self.availability_publisher = self._node.create_publisher(
            String,
            "availability",
            qos_profile=10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.activated = False

    def on_battery_full_callback(self):
        if not self.activated:
            msg = String()
            msg.data = self._prefix
            self.availability_publisher.publish(msg)  # Say "padflieID" if available.

    def activate(self) -> bool:
        if not self._charge_controller.is_charged():
            return False

        self._commander.activate()
        self.activated = True
        return True

    def deactivate(self):
        self._commander.deactivate()
        self.activated = False
