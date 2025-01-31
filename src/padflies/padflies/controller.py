from rclpy.node import Node

from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
    LoggingClient,
)

from ._padflie_tf import PadflieTF
from .connection_manager import ConnectionManager
from .charge_controller import ChargeController
from .commander import PadflieCommander

from typing import Callable
from crazyflies.crazyflie import CrazyflieType


class PadflieController:

    def __init__(
        self,
        node: Node,
        cf_type: CrazyflieType,
        prefix: str,
        hl_commander: HighLevelCommanderClient,
        ll_commander: GenericCommanderClient,
        log_commander: LoggingClient,
        tf_manager: PadflieTF,
        sleep: Callable[[float], None],
    ):
        self.__node = node
        self.__tf_manager = tf_manager
        self.__sleep = sleep

        # Fixed instance variables
        self.__world = "world"

        self._commander = PadflieCommander(
            node=node,
            prefix=prefix,
            hl_commander=hl_commander,
            ll_commander=ll_commander,
            tf_manager=tf_manager,
            sleep=sleep,
        )

        self._charge_controller = ChargeController(
            node=node, cf_type=cf_type, logging_client=log_commander
        )  # Checks charge state

        self._connection_manager = ConnectionManager(
            node=node,
            cf_type=cf_type,
            prefix=prefix,
            charge_controller=self._charge_controller,
            commander=self._commander,
        )  # Creates a connection to some controller, this way we get controlled from only one instance
