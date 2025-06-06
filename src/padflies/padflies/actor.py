from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped

from ._hl_commander_minimal import HighLevelCommander
from ._ll_commander_minimal import LowLevelCommander

from crazyflies.safe.safe_commander import SafeCommander
from .yaw_commander import YawCommander
from ._padflie_tf import PadflieTF


from enum import Enum, auto
from typing import List, Callable, Optional, Tuple

from collision_avoidance_interfaces.srv import CollisionAvoidance
import re
import numpy as np


class ActorState(Enum):
    DEACTIVATED = auto()
    LOW_LEVEL_COMMANDER = auto()
    HIGH_LEVEL_COMMANDER = auto()
    ERROR_STATE = auto()


class PadflieActor:
    def __init__(
        self,
        node: Node,
        tf_manager: PadflieTF,
        hl_commander: HighLevelCommander,
        ll_commander: LowLevelCommander,
        sleep: Callable[[float], None],
    ):
        self._node = node
        self._tf_manager = tf_manager
        self._hl_commander = hl_commander
        self._ll_commander = ll_commander
        self._sleep = sleep

        ### Instance variables
        self._state: ActorState = ActorState.DEACTIVATED
        self._fixed_yaw: bool = False

        self._control_rate: float = 10.0  # Hz
        self._dt: float = 1 / self._control_rate  # Loop period for LL Commander

        ### Commanders (non ROS)
        self.max_rotational_speed: float = 0.5  # Dependent on update rate??
        self.max_step_distance_xy: float = 3  # in meters/second
        self.max_step_distance_z: float = 1  # in meters/second
        self.clipping_box: Optional[List[float]] = None
        self.fixed_yaw_target: float = 0.0

        # TODO: This is from old isse_core for a stable flight
        self.target_history_size: int = 13
        self.__target_history: "list[list[float]]" = []

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

        # Control Variables, These should not be changed from outside.
        self.__target_pose: Optional[PoseStamped] = None
        self.__last_valid_target_position_and_yaw: Optional[
            Tuple[List[float], float]
        ] = None
        self.__current_yaw: float = (
            0.0  # We dont get yaw from cf. Assume this gets correctly tracked across flight
        )

    def activate(self):
        self._node.get_logger().info("Creating Publishers")
        self._hl_commander.create_publishers()
        self._ll_commander.create_publishers()

        self.__cmd_timer = self._node.create_timer(
            timer_period_sec=self._dt,
            callback=self.__send_target,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )  # This timer needs to be executed while we takeoff or land -> different callback group

        self._collision_avoidance_client = self._node.create_client(
            CollisionAvoidance,
            "collision_avoidance",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._state = ActorState.HIGH_LEVEL_COMMANDER

    def deactivate(self):
        self._node.destroy_timer(self.__cmd_timer)
        if self._state == ActorState.LOW_LEVEL_COMMANDER:
            self._hl_commander.land(0.0, 5.0)  # Failing semi safe
        self._hl_commander.destroy_publishers()
        self._ll_commander.destroy_publishers()
        self._node.destroy_client(self._collision_avoidance_client)

    def __calculate_collision_avoidance_target(
        self, position: "list[float]", target: "list[float]"
    ) -> Optional["list[float]"]:
        req = CollisionAvoidance.Request()
        req.id = int(re.search(r"\d+", self._hl_commander._prefix).group())
        req.position.x, req.position.y, req.position.z = position
        req.target.x, req.target.y, req.target.z = target
        fut = self._collision_avoidance_client.call_async(req)

        for i in range(10):
            if fut.done():
                break
            self._sleep(0.01)
        if fut.result() is None:
            return None
        res = fut.result()
        return [res.target.x, res.target.y, res.target.z]

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
            self._state is ActorState.LOW_LEVEL_COMMANDER
            and self.__target_pose is not None
        ):
            cf_position = self._tf_manager.get_cf_position()
            if cf_position is None:
                self._fail_safe("No Crazyflie position.")
                return

            target_position_and_yaw = (
                self._tf_manager.pose_stamped_to_world_position_and_yaw(
                    self.__target_pose
                )
            )

            target_position: List[float]
            target_yaw: float
            # Get target position and target yaw either from our target,
            #                                           the last valid target
            #                                           or the cf position
            if target_position_and_yaw is not None:
                # Target is ok.
                target_position, target_yaw = target_position_and_yaw
                self.__last_valid_target_position_and_yaw = target_position_and_yaw
            else:
                # Target not ok.
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

            if self._fixed_yaw:
                target_yaw = self.fixed_yaw_target

            # Collision avoidance.
            col_av_target_position = self.__calculate_collision_avoidance_target(
                cf_position, target_position
            )
            if col_av_target_position is not None:
                target_position = col_av_target_position

            # Average out targets (from old ISSE_CORE)
            self.__target_history.insert(0, target_position)
            self.__target_history = self.__target_history[: self.target_history_size]

            ln = len(self.__target_history)
            target_position = list(
                np.average(
                    self.__target_history,
                    axis=0,
                    weights=[((ln - i) / float(ln)) ** 2.6 for i in range(ln)],
                )
            )

            # Pass through safe commanders.
            safe_target = self._safe_commander.safe_cmd_position(
                cf_position, target_position
            )
            safe_yaw = self._yaw_commander.safe_cmd_yaw(self.__current_yaw, target_yaw)

            if self._state is ActorState.LOW_LEVEL_COMMANDER:
                # Due to timing this might happen.
                self._ll_commander.cmd_position(safe_target, safe_yaw)

            # Finally set our current yaw to the yaw we commanded
            # TODO: Is max_yawrate set low enough for this to be safe?
            self.__current_yaw = safe_yaw

    def _fail_safe(self, msg: str):
        self._state = ActorState.ERROR_STATE
        self._hl_commander.land(0.0, 5.0)
        self._node.get_logger().info(f"Transitioning to Error state. {msg}")

    def get_target_pose(self) -> Optional[PoseStamped]:
        return self.__target_pose

    def get_yaw(self):
        return self.__current_yaw

    def set_target(self, target: PoseStamped, use_yaw: bool):
        self.__target_pose = target
        self._fixed_yaw = not use_yaw

    def takeoff_routine(self, takeoff_height: float = 1.0) -> bool:
        """Routine for TAKEOFF.

        Takes off and puts this actor in LowLevel commander mode.
        This ensures that as soon as th crazyflie is in the air it is commanded with cmd_positions.

        Precondition: Crazyflie is in its Pad.
        After: Crazyflie hovers at takeoff_height above the pad. We are in LOW level commander Mode.

        Args:
            position (List[float]): The position we are currently at.
            takeoff_height (float, optional): We will idle at this height above the position after taking off. Defaults to 1.0.
        """
        if self._state is ActorState.ERROR_STATE:
            return False

        self._state = ActorState.HIGH_LEVEL_COMMANDER
        target_pose: Optional[PoseStamped] = self._tf_manager.get_pad_pose_world()
        if target_pose is None:
            return False
        target_pose.pose.position.z += takeoff_height

        self.set_target(
            target_pose, use_yaw=False
        )  # Set target so crazyflie hovers after takeoff

        self._hl_commander.go_to(
            x=0.0, y=0.0, z=0.1, yaw=0.0, duration_seconds=1.0, relative=True
        )
        self._sleep(0.5)
        self._hl_commander.go_to(
            x=0.0, y=0.0, z=0.6, yaw=0.0, duration_seconds=4, relative=True
        )

        # Even though we specified more time for takeoff, this ensures a cleaner transition.
        self._sleep(2)
        self._state = ActorState.LOW_LEVEL_COMMANDER
        return True

    def land_routine(self) -> bool:
        """Routine for LAND.

        Lands the crazyflie safely into the pad.

        Precondition: Crazyflie is in the air, not too far from the Pad (max. 0.75 Meters)
        After: Crazyflie is seated in the pad. Motors are off. State is HIGH Level Commander
        """
        if self._state is ActorState.ERROR_STATE:
            return

        self._ll_commander.notify_setpoints_stop(200)
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
        p_p_and_yaw = self._tf_manager.get_pad_position_and_yaw()

        self._state = ActorState.HIGH_LEVEL_COMMANDER
        if p_p_and_yaw is None:
            self._fail_safe("No pad position found for landing (Phase2a).")
            return False

        pad_position, yaw = p_p_and_yaw
        self._hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] + 0.25,
            yaw=yaw,
            duration_seconds=4.5,
        )
        self._sleep(4.5)  # Bleed of momentum we still have from target flight.

        p_p_and_yaw = self._tf_manager.get_pad_position_and_yaw()
        if p_p_and_yaw is None:
            self._fail_safe("No pad position found for landing (Phase2b).")
            return False

        pad_position, yaw = p_p_and_yaw
        self._hl_commander.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] - 0.1,
            yaw=yaw,
            duration_seconds=5.0,
        )
        self._sleep(5)  # The actual landing.

        """
        Phase3: 
            We are already in the Pad. 
            We land to stop motors and this Wiggles us to properly seat into the pad.
        """
        p_p_and_yaw = self._tf_manager.get_pad_position_and_yaw()
        if p_p_and_yaw is None:
            self._fail_safe("No pad position found for landing (Phase3).")
            return False

        pad_position, yaw = p_p_and_yaw
        self._hl_commander.land(
            target_height=-0.5, duration_seconds=2.5, yaw=yaw
        )  # Landing at height 0 ensures motors get shut off completely
        self._sleep(0.5)
        # Its ok to  launch again now, but landing rights need to be released!!!
        return True
