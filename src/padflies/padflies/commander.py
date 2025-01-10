from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
)
from crazyflies.safe.safe_commander import SafeCommander

from crazyflies_interfaces.msg import SendTarget
from std_msgs.msg import Empty

from typing import Callable, List
from enum import Enum, auto
import numpy as np
from copy import deepcopy


class PadFlieState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()
    TARGET_INTERNAL = auto()  # Do not accept external targets. But flie with target


class PadflieCommander:
    def __init__(
        self,
        node: Node,
        prefix: str,
        hl_commander: HighLevelCommanderClient,
        g_commander: GenericCommanderClient,
        get_position_callback: Callable[[], List[float]],
        get_pad_position_callback: Callable[[], List[float]],
        sleep_callback: Callable[[float], None],
    ):
        self.hl_commander = hl_commander
        self.ll_commander = g_commander
        self.get_position: Callable[[], List[float]] = get_position_callback
        self.get_pad_position: Callable[[], List[float]] = get_pad_position_callback
        self.sleep: Callable[[float],] = sleep_callback

        self.state: PadFlieState = PadFlieState.IDLE
        self.target: List[float] = None

        update_rate = 10.0  # Hz
        dt = 1 / update_rate

        self.commander = SafeCommander(
            dt=dt, max_step_distance_xy=3, max_step_distance_z=1, clipping_box=None
        )

        qos_profile = 10
        callback_group = MutuallyExclusiveCallbackGroup()

        node.create_timer(
            timer_period_sec=dt,
            callback=self.__send_target,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )  # This timer needs to be executed while we takeoff or land -> different callback group

        node.create_subscription(
            SendTarget,
            prefix + "/send_target",
            self._send_target_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            callback=self._takeoff_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        node.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            callback=self._land_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

    def _send_target_callback(self, msg: SendTarget) -> None:
        if self.state is not PadFlieState.TARGET_INTERNAL:
            x, y, z = msg.target.x, msg.target.y, msg.target.z
            self.target = [x, y, z]

    def _takeoff_callback(self, msg: Empty) -> None:
        if self.state is not PadFlieState.IDLE:
            return
        position = self.get_position()
        if position is not None:
            self.__takeoff_routine(position)
        else:
            raise Exception("Crazyflie doesnt have position. Cannot takeoff.")

    def _land_callback(self, msg: Empty) -> None:
        if self.state not in (PadFlieState.TARGET, PadFlieState.TARGET_INTERNAL):
            return
        pad_position = self.get_pad_position()
        if pad_position is None:
            raise Exception(
                "Could not find pad. Cannot land"
            )  # TODO: Dont crash but failsafe

        def get_pad_position():
            new_pad_position = self.get_pad_position()
            if new_pad_position is not None:
                pad_position = new_pad_position
                return pad_position
            return pad_position

        self.__land_routine(get_pad_position)

    def __send_target(self):
        if self.state not in (PadFlieState.TARGET, PadFlieState.TARGET_INTERNAL):
            return
        position = self.get_position()
        if position is not None and self.target is not None:
            safe_target = self.commander.safe_cmd_position(position, self.target)
            self.ll_commander.cmd_position(safe_target, 0.0)

    def __takeoff_routine(self, position):
        self.target = position
        self.target[2] += 1.0
        self.state = PadFlieState.TAKEOFF
        self.hl_commander.takeoff(target_height=self.target[2], duration_seconds=4.0)
        self.sleep(2.0)
        self.state = PadFlieState.TARGET

    def __land_routine(self, get_pad_position: Callable[[], List[float]]):
        """Phase 1:
        For at most 8 seconds.
        Use cmd_positions to target a point 0.5 meters above the pad.
        If position is reached continue with Phase2.

        If is is not reached PROBLEM TODO

        During this we need to update the pad position, maybe it moves.
        """
        self.state = PadFlieState.TARGET_INTERNAL
        timeout = 8.0
        while timeout > 0.0:
            pad_position = get_pad_position()
            self.target = deepcopy(pad_position)
            self.target[2] += 0.5
            pos = self.get_position()

            if (
                pos is not None
                and np.linalg.norm(np.array(pos) - np.array(self.target)) < 0.1
            ):  # closer than 5 cm
                break

            timeout -= 0.1
            self.sleep(0.1)
            pos = self.get_position()

        self.state = PadFlieState.LAND
        self.ll_commander.notify_setpoints_stop(100)

        """Phase2: 
        We are now only using HighLevelCommander
        Use GoTos to properly land. 
        We were 0.5 meters above the pad. 
        First Step: lower to 0.2 m above pad. This is because Phase1 probably overshoots
        Second Step: Fly straight down into the pad. For a good seating. Update Pad Positon alway.
        """

        pad_position = get_pad_position()
        self.hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] + 0.2,
            yaw=0.0,
            duration_seconds=2.0,
        )
        # TODO: Make yaw available
        self.sleep(3.0)  # momentum

        pad_position = get_pad_position()
        self.hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] - 0.1,
            yaw=0.0,
            duration_seconds=4.0,
        )
        self.sleep(3.0)

        """Phase3: 
        We are already in the Pad. 
        We land to stop motors and this Wiggles us to properly seat into the pad.
        """
        self.hl_commander.land(
            target_height=0.0, duration_seconds=3.0
        )  # Try to get into the pad (landing at pad_height would be more correct)
        self.sleep(2.5)
        self.state = PadFlieState.IDLE
