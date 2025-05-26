from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from padflies_interfaces.srv import GetBBoxes

from std_msgs.msg import Empty
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
        sleep: Callable[[float], None]
    ):
        self._node = node
        self._tf_manager = tf_manager
        self._hl_commander = hl_commander
        self._ll_commander = ll_commander
        self._sleep = sleep
        self._clipping_box = self._node.get_parameter("clipping_box").value

        ### Instance variables
        self._state: ActorState = ActorState.DEACTIVATED
        self._fixed_yaw: bool = False

        self._control_rate: float = 10.0  # Hz
        self._dt: float = 1 / self._control_rate  # Loop period for LL Commander

        ### Commanders (non ROS)
        self.max_rotational_speed: float = 0.5  # Dependent on update rate??
        self.max_step_distance_xy: float = 3  # in meters/second
        self.max_step_distance_z: float = 1  # in meters/second
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
            clipping_box=self._clipping_box,
        )

        # for no fly zones
        _bbox_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._node.create_subscription(
            msg_type=Empty,
            topic="/update_bboxes",
            callback=self._update_bboxes_callback,
            qos_profile=_bbox_qos_profile,
        )

        # Control Variables, These should not be changed from outside.
        self.__target_pose: Optional[PoseStamped] = None
        self.__last_valid_target_position_and_yaw: Optional[
            Tuple[List[float], float]
        ] = None
        self.__current_yaw: float = 0.0  # We dont get yaw from cf. Assume this gets correctly tracked across flight

        self.has_collision_avoidance_client = False

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
        self.has_collision_avoidance_client = True

        self._state = ActorState.HIGH_LEVEL_COMMANDER

    def deactivate(self):
        self._node.destroy_timer(self.__cmd_timer)
        if self._state == ActorState.LOW_LEVEL_COMMANDER:
            self._hl_commander.land(0.0, 5.0)  # Failing semi safe

        self._hl_commander.destroy_publishers()
        self._ll_commander.destroy_publishers()

        self.has_collision_avoidance_client = False
        self._node.destroy_client(self._collision_avoidance_client)

    def __calculate_collision_avoidance_target(
        self, position: "list[float]", target: "list[float]"
    ) -> Optional["list[float]"]:
        req = CollisionAvoidance.Request()
        req.id = int(re.search(r"\d+", self._hl_commander._prefix).group())
        req.position.x, req.position.y, req.position.z = position
        req.target.x, req.target.y, req.target.z = target

        if self.has_collision_avoidance_client:
            fut = self._collision_avoidance_client.call_async(req)
            for i in range(10):
                if fut.done():
                    break
                self._sleep(0.01)
            if fut.result() is None:
                return None
            res = fut.result()

            return [res.target.x, res.target.y, res.target.z]
        return None

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
                    (
                        target_position,
                        target_yaw,
                    ) = self.__last_valid_target_position_and_yaw
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

    def set_bboxes(self, names:List[str], centers: List[List[float]], rotations:List[List[float]], sizes:List[List[float]]) -> None:
        # or do this somewhere else as this is being triggered from outside..
        target = self.get_target_pose()
        if target:
            target = [target.pose.position.x, target.pose.position.y, target.pose.position.z]
            force, inside_mask = point_in_any_box(target, centers, rotations, sizes)

            self._node.get_logger().error("Force to move outside the boxes " + str(force))

    def _update_bboxes_callback(self, msg: Empty) -> None:
        """
        Requests new bboxes from no_fly_zone_manager.
        Important: The transformation received is in world coordinates!
        Then calls update_bboxes of self
        """
        def update_bboxes(response):
            result = response.result()

            names = []
            centers = []
            rotations = []
            sizes = []

            for obj in result.bboxes:
                names.append(str(obj.transform.child_frame_id))
                center = obj.transform.transform.translation
                centers.append([center.x, center.y, center.z])
                rot =  obj.transform.transform.rotation
                rotations.append([rot.x, rot.y, rot.z, rot.w])
                size = obj.size
                sizes.append([size.x, size.y, size.z])
            
            self.set_bboxes(names, centers, rotations, sizes)

        client = self._node.create_client(
            srv_type=GetBBoxes,
            srv_name="get_no_fly_zones"
        )

        if client.wait_for_service(timeout_sec=1.0): # TODO Winni, ist das doof?
            req = GetBBoxes.Request()
            bbox_future = client.call_async(req)
            bbox_future.add_done_callback(update_bboxes)


def quaternion_to_rotation_matrix_batch(quaternion_batch):
    """
    Converts a list of quaternions with shape (N, 4) into rotation matrices with shape (N, 3, 3).
    :param quaternion_batch: (N, 4) rotations
    """
    q = np.asarray(quaternion_batch)
    w, x, y, z = q[:,0], q[:,1], q[:,2], q[:,3]

    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2*(y**2 + z**2)
    R[:, 0, 1] = 2*(x*y - z*w)
    R[:, 0, 2] = 2*(x*z + y*w)

    R[:, 1, 0] = 2*(x*y + z*w)
    R[:, 1, 1] = 1 - 2*(x**2 + z**2)
    R[:, 1, 2] = 2*(y*z - x*w)

    R[:, 2, 0] = 2*(x*z - y*w)
    R[:, 2, 1] = 2*(y*z + x*w)
    R[:, 2, 2] = 1 - 2*(x**2 + y**2)
    return R

def point_in_any_box(point, center_batch, quaternion_batch, size_batch):
    """
    Checks if a point is inside of any of the given rotated boxes

    :param point: (3,) Point to check
    :param center_batch: (N, 3) centers
    :param quaternion_batch: (N, 4) rotations
    :param size_batch: (N, 3) sizes
    :return: Bool-Array (N,) → True for every box the point is inside of
    """
    p = np.asarray(point)
    centers = np.asarray(center_batch)
    sizes = np.asarray(size_batch)
    half_sizes = sizes / 2
    quats = np.asarray(quaternion_batch)

    # Calculate rotation matrices of all rotations
    R = quaternion_to_rotation_matrix_batch(quats)  # (N, 3, 3)

    # Perform a batched transformation of vectors from global/world coordinates into local coordinates 
    # by applying the transpose (i.e., inverse) of rotation matrices.
    relative = p - centers  # (N, 3)
    local = np.einsum('nij,nj->ni', R.transpose(0, 2, 1), relative)

    # no check if our point is in any box
    inside_mask =  np.all(np.abs(local) <= half_sizes, axis=1)

    force = np.zeros(3)

    for i, inside in enumerate(inside_mask):
        if not inside:
            continue
        # Compute vector to nearest box face in local space
        delta = half_sizes[i] - np.abs(local[i])  # distance to each face
        min_axis = np.argmin(delta)              # closest face
        direction = np.zeros(3)
        direction[min_axis] = np.sign(local[i][min_axis])  # push outward along that axis

        exit_local = direction * delta[min_axis]

        # Rotate back to world frame
        exit_world = R[i] @ exit_local
        force += exit_world

    return force, inside_mask