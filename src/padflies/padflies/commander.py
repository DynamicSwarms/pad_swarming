from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher

from std_msgs.msg import Empty
from crazyflies_interfaces.msg import SendTarget

from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
    LoggingClient,
)
from crazyflies.safe.safe_commander import SafeCommander

from ._padflie_states import PadFlieState
from .charge_controller import ChargeController
from .actor import PadflieActor

from copy import deepcopy
import numpy as np
from typing import Callable, List, Optional, Tuple


class PadflieCommander:

    def __init__(
        self,
        node: Node,
        prefix: str,
        hl_commander: HighLevelCommanderClient,
        g_commander: GenericCommanderClient,
        log_commander: LoggingClient,
        get_position_callback: Callable[[], Optional[List[float]]],
        get_pad_position_and_yaw_callback: Callable[[], Optional[List[float]]],
        sleep_callback: Callable[[float], None],
    ):
        self.__node = node
        self.__get_position: Callable[[], Optional[List[float]]] = get_position_callback
        self.__get_pad_position_and_yaw: Callable[
            [], Optional[Tuple[List[float], float]]
        ] = get_pad_position_and_yaw_callback

        self._sleep: Callable[[float], None] = sleep_callback
        self._prefix = prefix
        self._state: PadFlieState = PadFlieState.IDLE

        target_rate: float = 10.0  # Hz
        dt: float = 1 / target_rate
        self.commander = SafeCommander(
            dt=dt, max_step_distance_xy=3, max_step_distance_z=1, clipping_box=None
        )

        self._actor = PadflieActor(
            node=node,
            hl_commander=hl_commander,
            ll_commander=g_commander,
            sleep=sleep_callback,
            dt=dt,
        )
        self._charge_controller = ChargeController(
            node=node, logging_client=log_commander
        )

        callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # All subscriptions can be on the same callbackgroup
        qos_profile = 10
        node.create_subscription(
            SendTarget,
            prefix + "/send_target",
            self.__send_target_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            callback=self.__takeoff_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            callback=self.__land_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def set_target(self, target: List[float]) -> None:
        """Calculates a safe target and sets it as our target.

        Args:
            target (List[float]): The target to follow.
        """
        position = self.__get_position()
        if position is not None:
            safe_target = self.commander.safe_cmd_position(position, target)
            self._actor.set_target(safe_target)

    def __send_target_callback(self, msg: SendTarget):
        """Callback to the send_target topic.

        TODO: Use the topics frame_id and conversions to handle this nicely.

        Args:
            msg (SendTarget): An empty message used as trigger.
        """
        if self._state is not PadFlieState.TARGET:
            return

        self.set_target([msg.target.x, msg.target.y, msg.target.z])

    def __takeoff_callback(self, msg: Empty):
        """Callback to the takeoff subscription.

        The crazyflie will only takeoff if it knows its position.

        This routine takes exactly 2 seconds to complete
        Args:
            msg (Empty): An Empty message. Just used as trigger.
        """
        if self._state is not PadFlieState.IDLE:
            return

        position = self.__get_position()
        if position is None:
            self.__node.get_logger().error(
                "Crazyflie doesnt have position. Cannot takeoff."
            )
            return

        self._state = PadFlieState.TAKEOFF
        self._actor.takeoff_routine(position)
        self._state = PadFlieState.TARGET

    def __land_callback(self, msg: Empty):
        """Callback to the land subscription.

        The crazyflie will only land if it knows its own position and the position of its landing pad.

        If the position and pad_position is valid is only checked once.
        After that landing will try to get updated positions but receives old ones if there is an issue.

        The routine transitions us through TARGET_INTERNAL, LAND and leaves with state IDLE
        It takes 16.5 seconds

        TODO: Maybe there are strategies to handle this better. But if there is no position available something is bad anyway.
        Args:
            msg (Empty): An empty message. Just used as trigger.
        """
        if self._state is not PadFlieState.TARGET:
            return

        pad_position_and_yaw: Optional[Tuple[List[float], float]] = (
            self.__get_pad_position_and_yaw()
        )
        position: Optional[List[float]] = self.__get_position()

        if pad_position_and_yaw is None:
            self.__node.get_logger().error("Could not find pad. Cannot land")
            return
        if position is None:
            self.__node.get_logger().error("Was not able to find the position.")
            return

        def get_pad_position_and_yaw():
            new_pad_position_and_yaw = self.__get_pad_position_and_yaw()
            if new_pad_position_and_yaw is not None:
                pad_position_and_yaw = new_pad_position_and_yaw
            return deepcopy(pad_position_and_yaw)

        def get_position():
            new_position = self.__get_position()
            if new_position is not None:
                position = new_position
            return deepcopy(position)

        self._state = PadFlieState.LAND
        """
        Phase 1:
            For at most 8 seconds.
            Fly with safe targets to a point 0.5 meters above the pad.
            If position is reached continue with Phase2.

            If is is not reached PROBLEM TODO

            During this we need to update the pad position, maybe it moves.

            Phases 2 and 3 are in the routine.
        """
        timeout = 8.0
        while timeout > 0.0:
            target, yaw = get_pad_position_and_yaw()
            target[2] += 0.5
            self.set_target(target)

            if (
                np.linalg.norm(np.array(get_position()) - np.array(target)) < 0.1
            ):  # closer than 10 cm
                break

            timeout -= 0.1
            self._sleep(0.1)

        self._actor.land_routine(get_pad_position_and_yaw=get_pad_position_and_yaw)
        self._state = PadFlieState.IDLE
