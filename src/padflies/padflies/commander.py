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


class PadflieCommander:

    def __init__(
        self,
        node: Node,
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
        self._state: PadFlieState = PadFlieState.IDLE
        self.__world = "world"

        self._actor = PadflieActor(
            node=node,
            tf_manager=tf_manager,
            hl_commander=hl_commander,
            ll_commander=ll_commander,
            sleep=sleep,
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

    def _set_target(self, target: PoseStamped, use_yaw: bool = False):
        """Set a PoseStamped as the actor target.

        Args:
            target (PoseStamped): The target for the crazyflie.
            use_yaw (bool): Wheter to use the yaw in the pose (True) or set it to 0 (False)
        """
        self._actor.set_target(target, use_yaw)

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

        self._set_target(msg.target, msg.use_yaw)

    def __takeoff_callback(self, msg: Empty):
        """Callback to the takeoff subscription.

        The crazyflie will only takeoff if it knows its position.

        This routine takes exactly 2 seconds to complete
        Args:
            msg (Empty): An Empty message. Just used as trigger.
        """
        if self._state is not PadFlieState.IDLE:
            return

        position = self.__tf_manager.get_cf_position()
        if position is None:
            self.__node.get_logger().error(
                "Crazyflie doesnt have position. Cannot takeoff."
            )
            return

        self._state = PadFlieState.TAKEOFF
        self._actor.takeoff_routine()
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
            target_pose = self.__tf_manager.get_pad_pose()
            target_pose.pose.position.z += 0.5
            self._set_target(target=target_pose, use_yaw=True)

            cf_position = self.__tf_manager.get_cf_position()
            pad_target = self.__tf_manager.pose_stamped_to_world_position_and_yaw(
                target_pose
            )
            if cf_position is None or pad_target is None:
                self.__node.get_logger().info(
                    "Big problem with transforms flying home."
                )
                self.__sleep(0.1)
                continue

            target, _yaw = pad_target
            if (
                np.linalg.norm(np.array(cf_position) - np.array(target)) < 0.1
            ):  # closer than 10 cm
                break

            timeout -= 0.1
            self.__sleep(0.1)

        self._actor.land_routine()
        self._state = PadFlieState.IDLE
