from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from padflies_interfaces.msg import SendTarget

from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
    LoggingClient,
)
from crazyflies.safe.safe_commander import SafeCommander

from ._padflie_states import PadFlieState
from ._qos_profiles import qos_profile_simple
from ._padflie_tf import PadflieTF
from .connection_manager import ConnectionManager
from .charge_controller import ChargeController
from .actor import PadflieActor

import tf_transformations
from copy import deepcopy
import numpy as np
from typing import Callable, List, Optional, Tuple


class PadflieCommander(SafeCommander):
    _state: PadFlieState = PadFlieState.IDLE
    __control_rate: float = 10.0  # Hz
    _dt: float = 1 / __control_rate

    __world = "world"

    def __init__(
        self,
        node: Node,
        prefix: str,
        hl_commander: HighLevelCommanderClient,
        ll_commander: GenericCommanderClient,
        log_commander: LoggingClient,
        tf_manager: PadflieTF,
        get_position: Callable[[], Optional[List[float]]],
        sleep: Callable[[float], None],
    ):
        super().__init__(
            dt=self._dt,
            max_step_distance_xy=3,
            max_step_distance_z=1,
            clipping_box=None,
        )
        self.__node = node
        self.__tf_manager = tf_manager
        self.__get_position = get_position
        self.__sleep = sleep

        self._actor = PadflieActor(
            node=node,
            ckeck_target=self.check_target,
            hl_commander=hl_commander,
            ll_commander=ll_commander,
            sleep=sleep,
            dt=self._dt,
        )  # This actor is allowed to send control commands to the crazyflie

        self._charge_controller = ChargeController(
            node=node, logging_client=log_commander
        )  # Checks charge state

        self._connection_manager = ConnectionManager(
            node=node, prefix=prefix
        )  # Creates a connection to some controller, this way we get controlled from only one instance

        #########
        # Initialization of our own Subscriptions to command position, takeoff and land
        #########

        callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # All subscriptions can be on the same callbackgroup
        node.create_subscription(
            SendTarget,
            prefix + "/send_target",
            self.__send_target_callback,
            qos_profile=qos_profile_simple,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            callback=self.__takeoff_callback,
            qos_profile=qos_profile_simple,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            callback=self.__land_callback,
            qos_profile=qos_profile_simple,
            callback_group=callback_group,
        )

    def check_target(self, target: List[float]) -> List[float]:
        position = self.__get_position()
        # TODO: What to do here, fail safe
        if position is not None:
            return self.safe_cmd_position(position, target)

    def _set_target(self, target: List[float], yaw: Optional[float] = None) -> None:
        """Calculates a safe target and sets it as our target.

        Args:
            target (List[float]): The target to follow.
        """

        self._actor.set_target(target)

    def __send_target_callback(self, msg: SendTarget):
        """Callback to the send_target topic.

        Args:
            msg (SendTarget): An empty message used as trigger.
        """
        if (
            self._state is not PadFlieState.TARGET
            or msg.priority_id < self._connection_manager.current_priority_id
        ):
            return

        _world_target: PoseStamped = self.__tf_manager.transform_pose_stamped(
            msg.target, self.__world, self.__node.get_logger()
        )
        self.__node.get_logger().info(str(_world_target) + str(msg.target))
        if _world_target is not None:
            world_target = [
                _world_target.pose.position.x,
                _world_target.pose.position.y,
                _world_target.pose.position.z,
            ]

            yaw: Optional[float] = None
            if msg.use_yaw:
                _roll, _pitch, yaw = tf_transformations.euler_from_quaternion(
                    tuple(
                        _world_target.pose.orientation.x,
                        _world_target.pose.orientation.y,
                        _world_target.pose.orientation.z,
                        _world_target.pose.orientation.w,
                    )
                )

            self._set_target(world_target, yaw)

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
            self.__tf_manager.get_pad_position_and_yaw()
        )
        position: Optional[List[float]] = self.__get_position()

        if pad_position_and_yaw is None:
            self.__node.get_logger().error("Could not find pad. Cannot land")
            return
        if position is None:
            self.__node.get_logger().error("Was not able to find the position.")
            return

        def get_pad_position_and_yaw():
            new_pad_position_and_yaw = self.__tf_manager.get_pad_position_and_yaw()
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
            self._set_target(target, yaw)

            if (
                np.linalg.norm(np.array(get_position()) - np.array(target)) < 0.1
            ):  # closer than 10 cm
                break

            timeout -= 0.1
            self.__sleep(0.1)

        self._actor.land_routine(get_pad_position_and_yaw=get_pad_position_and_yaw)
        self._state = PadFlieState.IDLE
