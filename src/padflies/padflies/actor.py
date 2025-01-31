from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped

from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
)

from crazyflies.safe.safe_commander import SafeCommander
from .yaw_commander import YawCommander
from ._padflie_tf import PadflieTF


from enum import Enum, auto
from typing import List, Callable, Optional, Tuple


class ActorState(Enum):
    LOW_LEVEL_COMMANDER = auto()
    HIGH_LEVEL_COMMANDER = auto()
    ERROR_STATE = auto()


class PadflieActor:
    def __init__(
        self,
        node: Node,
        tf_manager: PadflieTF,
        hl_commander: HighLevelCommanderClient,
        ll_commander: GenericCommanderClient,
        sleep: Callable[[float], None],
    ):
        self.__node = node
        self.__tf_manager = tf_manager
        self.__hl_commander = hl_commander
        self.__ll_commander = ll_commander
        self.__sleep = sleep

        ### Instance variables
        self.__state: ActorState = ActorState.HIGH_LEVEL_COMMANDER
        self.__target_pose: Optional[PoseStamped] = None
        self.__fixed_yaw: bool = False

        self.__last_valid_target_position_and_yaw: Optional[
            Tuple[List[float], float]
        ] = None

        self.__current_yaw: float = (
            0.0  # We dont get yaw from cf. Assume this gets correctly tracked across flight
        )

        self.__control_rate: float = 10.0  # Hz
        self._dt: float = 1 / self.__control_rate

        self.max_rotational_speed: float = 0.5  # Dependent on update rate??
        self.max_step_distance_xy: float = 3  # in meters/second
        self.max_step_distance_z: float = 1  # in meters/second
        self.clipping_box: Optional[List[float]] = None
        self.fixed_yaw_target: float = 0.0

        ### Commanders and ROS-functionality

        self._yaw_commander = YawCommander(
            dt=self._dt,
            max_rotational_speed=self.max_rotational_speed,
        )
        self._safe_commander = SafeCommander(
            dt=self._dt,
            max_step_distance_xy=self.max_step_distance_xy,
            max_step_distance_z=self.max_step_distance_z,
            clipping_box=self.clipping_box,
        )

        node.create_timer(
            timer_period_sec=self._dt,
            callback=self.__send_target,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )  # This timer needs to be executed while we takeoff or land -> different callback group

    def __send_target(self):
        """Callback to the LOW level commander timer.

        Sends low level commands to the crazyflie if:
                In LOW level commander
            and A target is set

        This is the case after a takeoff.
        If there is no valid crazyflie position we failsafe.
        If there is no transform to the target pose available we choose the last valid one.
        If there was no last valid target position we choose crazyflie position in order to hover.

        After a target is choosen it passes through safe_commanders in order not to crash the crazyflie.
        This is because the commanded positions are not checked before sent to kalman onboard crazyflie.
        """
        if (
            self.__state is ActorState.LOW_LEVEL_COMMANDER
            and self.__target_pose is not None
        ):
            cf_position = self.__tf_manager.get_cf_position()
            if cf_position is None:
                self.__fail_safe("No Crazyflie position.")
                return

            target_position_and_yaw = (
                self.__tf_manager.pose_stamped_to_world_position_and_yaw(
                    self.__target_pose
                )
            )

            target_position: List[float]
            target_yaw: float
            # Get target position and target yaw either from our target,
            #                                           the last valid target
            #                                           or the cf position
            if target_position_and_yaw is not None:
                target_position, target_yaw = target_position_and_yaw

                self.__last_valid_target_position_and_yaw = target_position_and_yaw
            else:
                # TODO: Should we hover or should we follow last valid target??
                if self.__last_valid_target_position_and_yaw is not None:
                    target_position, target_yaw = (
                        self.__last_valid_target_position_and_yaw
                    )
                else:
                    # Lets hover in place. This case is probably rare.
                    target_position = cf_position
                    target_yaw = (
                        self.fixed_yaw_target
                    )  # This also might indicate to user that something is not right

            if self.__fixed_yaw:
                target_yaw = self.fixed_yaw_target

            # Pass through safe commander.
            safe_target = self._safe_commander.safe_cmd_position(
                cf_position, target_position
            )
            safe_yaw = self._yaw_commander.safe_cmd_yaw(self.__current_yaw, target_yaw)
            self.__ll_commander.cmd_position(safe_target, safe_yaw)

            self.__current_yaw = safe_yaw  # Finally set our current yaw to the yaw we commanded TODO: Is max_yawrate set low enough for this to be safe?

    def __fail_safe(self, msg: str):
        self.__state = ActorState.ERROR_STATE
        self.__hl_commander.land(0.0, 5.0)
        self.__node.get_logger().info(f"Transitioning to Error state. {msg}")

    def set_target(self, target: PoseStamped, use_yaw: bool):
        self.__target_pose = target
        self.__fixed_yaw = not use_yaw

    def takeoff_routine(self, takeoff_height: float = 1.0):
        """Routine for TAKEOFF.

        Takes off and puts this actor in LowLevel commander mode.
        This ensures that as soon as th crazyflie is in the air it is commanded with cmd_positions.

        Precondition: Crazyflie is in its Pad.
        After: Crazyflie hovers at takeoff_height above the pad. We are in LOW level commander Mode.

        Args:
            position (List[float]): The position we are currently at.
            takeoff_height (float, optional): We will idle at this height above the position after taking off. Defaults to 1.0.
        """
        if self.__state is ActorState.ERROR_STATE:
            return

        self.__state = ActorState.HIGH_LEVEL_COMMANDER
        target_pose: Optional[PoseStamped] = self.__tf_manager.get_pad_pose_world()
        if target_pose is None:
            return
        target_pose.pose.position.z += takeoff_height

        self.set_target(
            target_pose, use_yaw=False
        )  # Set target so crazyflie hovers after takeoff

        self.__hl_commander.takeoff(
            target_height=target_pose.pose.position.z,
            duration_seconds=4.0,
            yaw=0.0,
        )
        self.__sleep(
            2.0
        )  # Even though we specified 5 seconds for takeoff, this ensures a cleaner transition.
        self.__state = ActorState.LOW_LEVEL_COMMANDER

    def land_routine(self):
        """Routine for LAND.

        Lands the crazyflie safely into the pad.

        Precondition: Crazyflie is in the air, not too far from the Pad (max. 0.75 Meters)
        After: Crazyflie is seated in the pad. Motors are off. State is HIGH Level Commander
        """
        if self.__state is ActorState.ERROR_STATE:
            return

        self.__state = ActorState.HIGH_LEVEL_COMMANDER
        self.__ll_commander.notify_setpoints_stop(100)
        """
        Phase2: 
            We are now only using HighLevelCommander
            Use GoTos to properly land. 
            We were 0.5 meters above the pad. 
            First Step: lower to 0.2 m above pad. This is because Phase1 probably overshoots
            Second Step: Fly straight down into the pad. For a good seating.

            During this phase a moving pad is bad but we need the clean high level command routines for safe flight. 

            Between each command the pad position gets querried again.
        """
        p_p_and_yaw = self.__tf_manager.get_pad_position_and_yaw()
        if p_p_and_yaw is None:
            self.__fail_safe("No pad position found for landing (Phase2a).")
            return

        pad_position, yaw = p_p_and_yaw
        self.__hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] + 0.2,
            yaw=yaw,
            duration_seconds=2.0,
        )
        self.__sleep(3.0)  # Bleed of momentum we still have from target flight.

        p_p_and_yaw = self.__tf_manager.get_pad_position_and_yaw()
        if p_p_and_yaw is None:
            self.__fail_safe("No pad position found for landing (Phase2b).")
            return

        pad_position, yaw = p_p_and_yaw
        self.__hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] - 0.1,
            yaw=yaw,
            duration_seconds=4.0,
        )
        self.__sleep(3.0)  # The actual landing.

        """
        Phase3: 
            We are already in the Pad. 
            We land to stop motors and this Wiggles us to properly seat into the pad.
        """
        p_p_and_yaw = self.__tf_manager.get_pad_position_and_yaw()
        if p_p_and_yaw is None:
            self.__fail_safe("No pad position found for landing (Phase3).")
            return

        pad_position, yaw = p_p_and_yaw
        self.__hl_commander.land(
            target_height=0.0, duration_seconds=3.0, yaw=yaw
        )  # Landing at height 0 ensures motors get shut off completely
        self.__sleep(2.5)  # It would be ok to launch after only 2.5 seconds
