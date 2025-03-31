from rclpy.node import Node
from enum import Enum, auto
from .padflie_commander import PadflieCommander
from .padflie_connector import PadflieConnector
from geometry_msgs.msg import PoseStamped
import time

from typing import Optional
from padflies._padflie_states import PadFlieState


class FlieState(Enum):
    IDLE = auto()
    FLYING = auto()


class PadflieConnection:
    def __init__(self, node: Node, reference_tf: str):
        self._node = node
        self._reference_tf = reference_tf
        self.__state = FlieState.IDLE

        self._commander = PadflieCommander(node=self._node)

        self._connector = PadflieConnector(
            node=self._node,
            on_connect_callback=self.on_connect,
            on_disconnect_callback=self.on_disconnect,
        )

        self._node.create_timer(1.0, self.check_battery)

    def get_padflie_state(self) -> Optional[PadFlieState]:
        return self._commander.get_padflie_state()

    def set_searching(self):
        self._connector.start_search()

    def is_searching(self):
        return self._connector.is_active()

    def is_active(self):
        return self._connector.is_active()

    def set_stop(self):
        # Also disconnects existing one
        self._connector.stop_search()

    def is_connected(self) -> bool:
        return self._connector.is_connected()

    def get_local_position(self) -> "list[float]":
        # Returns 0,0,0 if not yet has set
        pos = self._commander.get_local_position()
        if pos is not None:
            return pos
        return [0.0, 0.0, 0.0]

    def set_local_target(self, target: "list[float]"):
        target_msg = PoseStamped()
        target_msg.header.frame_id = self._reference_tf
        (
            target_msg.pose.position.x,
            target_msg.pose.position.y,
            target_msg.pose.position.z,
        ) = target

        self._commander.send_target(target_msg)

    def check_battery(self):
        if self.__state == FlieState.FLYING:
            if self._commander.battery_is_empty():
                self._connector.disconnect()  # Gets replaced

    def on_connect(self, prefix: str):
        """
        Connector module found a crazyflie to connect to.
        We connect the commander, takeoff and fly
        """
        self._commander.connect(prefix)
        time.sleep(0.3)
        self._commander.takeoff()
        self.__state = FlieState.FLYING

    def on_disconnect(self, prefix: str):
        """
        Connector module says its disconnected.
        Either because we landed and triggered it or there was an error in padflie.
        We disconnect the commander module and are ready to be connected again.
        """
        self.__state = FlieState.IDLE
        self._commander.disconnect()
