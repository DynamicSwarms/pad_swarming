from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
)

from copy import deepcopy
import numpy as np

from enum import Enum, auto
from typing import List, Callable, Optional


class ActorState(Enum):
    LOW_LEVEL_COMMANDER = auto()
    HIGH_LEVEL_COMMANDER = auto()


class PadflieActor:
    def __init__(
        self,
        node: Node,
        hl_commander: HighLevelCommanderClient,
        ll_commander: GenericCommanderClient,
        sleep: Callable[[float], None],
        dt: float,
    ):
        self.__hl_commander = hl_commander
        self.__ll_commander = ll_commander
        self.__sleep = sleep

        self.__state = ActorState.HIGH_LEVEL_COMMANDER
        self.__target_setpoint: Optional[List[float]] = None
        self.__yaw_setpoint: Optional[float] = None

        node.create_timer(
            timer_period_sec=dt,
            callback=self.__send_target,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )  # This timer needs to be executed while we takeoff or land -> different callback group

    def __send_target(self):
        """Callback to the low level commander timer.

        Sends low level commands to the crazyflie if in the air and in Low level mode.
        """
        if (
            self.__state is ActorState.LOW_LEVEL_COMMANDER
            and self.__target_setpoint is not None
            and self.__yaw_setpoint is not None
        ):
            self.__ll_commander.cmd_position(
                self.__target_setpoint, self.__yaw_setpoint
            )

    def set_target(self, target: List[float], yaw: float = 0.0):
        self.__target_setpoint = target
        self.__yaw_setpoint = yaw

    def takeoff_routine(
        self, position: List[float], takeoff_height: float = 1.0, yaw: float = 0.0
    ):
        self.__state = ActorState.HIGH_LEVEL_COMMANDER
        self.__target_setpoint = position
        self.__target_setpoint[2] += takeoff_height
        self.__yaw_setpoint = yaw

        self.__hl_commander.takeoff(
            target_height=self.__target_setpoint[2],
            duration_seconds=4.0,
            yaw=self.__yaw_setpoint,
        )
        self.__sleep(2.0)
        self.__state = ActorState.LOW_LEVEL_COMMANDER

    def land_routine(
        self,
        get_pad_position_and_yaw: Callable[[], List[float]],
    ):
        self.__state = ActorState.HIGH_LEVEL_COMMANDER
        self.__ll_commander.notify_setpoints_stop(100)
        """
        Phase2: 
            We are now only using HighLevelCommander
            Use GoTos to properly land. 
            We were 0.5 meters above the pad. 
            First Step: lower to 0.2 m above pad. This is because Phase1 probably overshoots
            Second Step: Fly straight down into the pad. For a good seating. Update Pad Positon alway.

            During this phase a moving pad is not good
        """

        pad_position, yaw = get_pad_position_and_yaw()
        self.__hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] + 0.2,
            yaw=yaw,
            duration_seconds=2.0,
        )
        self.__sleep(3.0)  # bleed off momentum

        pad_position, yaw = get_pad_position_and_yaw()
        self.__hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] - 0.1,
            yaw=yaw,
            duration_seconds=4.0,
        )
        self.__sleep(3.0)

        """
        Phase3: 
            We are already in the Pad. 
            We land to stop motors and this Wiggles us to properly seat into the pad.
        """
        pad_position, yaw = get_pad_position_and_yaw()
        self.__hl_commander.land(
            target_height=0.0, duration_seconds=3.0, yaw=yaw
        )  # Try to get into the pad (landing at pad_height would be more correct)
        self.__sleep(2.5)
