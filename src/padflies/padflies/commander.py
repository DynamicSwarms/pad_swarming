from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.task import Future

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from padflies_interfaces.msg import SendTarget, PadflieInfo
from pad_management_interfaces.srv import PadRightAcquire, PadRightRelease

from ._hl_commander_minimal import HighLevelCommander
from ._ll_commander_minimal import LowLevelCommander

from ._padflie_states import PadFlieState
from ._qos_profiles import qos_profile_simple
from ._padflie_tf import PadflieTF
from .actor import PadflieActor
import numpy as np
from typing import Callable, Optional
from scipy.spatial.transform import Rotation


class PadflieCommander:

    def __init__(
        self,
        node: Node,
        prefix: str,
        cf_prefix: str,
        tf_manager: PadflieTF,
        sleep: Callable[[float], None],
    ):
        self._node = node
        self._prefix = prefix
        self._cf_prefix = cf_prefix
        self._sleep = sleep
        self._tf_manager = tf_manager

        self._state = PadFlieState.DEACTIVATED
        # self._current_priority_id = 0 # Gets increased in every activation.

        self._actor = PadflieActor(
            node=node,
            tf_manager=tf_manager,
            hl_commander=HighLevelCommander(node=self._node, prefix=self._cf_prefix),
            ll_commander=LowLevelCommander(node=self._node, prefix=self._cf_prefix),
            sleep=sleep,
        )  # This actor is allowed to send control commands to the crazyflie


        self._callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # All subscriptions can be on the same callbackgroup
        
        self._landing_rights_callback_group = MutuallyExclusiveCallbackGroup()

        self.has_subscriptions = False

    def activate(self):
        self._actor.activate()

        self._state = PadFlieState.IDLE
        # self._current_priority_id += 1

        self.__acquire_pad_rights_client = self._node.create_client(
            srv_type=PadRightAcquire,
            srv_name="acquire_pad_right",
            callback_group=self._landing_rights_callback_group)
        
        self.__release_pad_rights_client = self._node.create_client(
            srv_type=PadRightRelease,
            srv_name="release_pad_right",
            callback_group=self._landing_rights_callback_group)

        self.__send_target_sub = self._node.create_subscription(
            SendTarget,
            self._prefix + "/send_target",
            self.__send_target_callback,
            qos_profile=qos_profile_simple,
            callback_group=self._callback_group,
        )

        self.__takeoff_sub = self._node.create_subscription(
            msg_type=Empty,
            topic=self._prefix + "/pad_takeoff",
            callback=self.__takeoff_callback,
            qos_profile=qos_profile_simple,
            callback_group=self._callback_group,
        )

        self.__land_sub = self._node.create_subscription(
            msg_type=Empty,
            topic=self._prefix + "/pad_land",
            callback=self.__land_callback,
            qos_profile=qos_profile_simple,
            callback_group=self._callback_group,
        )

        self.__info_pub = self._node.create_publisher(
            msg_type=PadflieInfo,
            topic=self._prefix + "/info",
            qos_profile=qos_profile_simple,
            callback_group=self._callback_group,
        )

        self.__info_timer = self._node.create_timer(0.1, self._info_timer_callback)



        self.has_subscriptions = True

    def deactivate(self):
        """Deactivating. 
        
        This closes all subscriptions and lands if neccesarry.
        Close subscribers first and then do smth about beeing still in the air. 
        This ensures nobody else will mess with us in the meantime.
        """
        if self.has_subscriptions:
            self._node.destroy_subscription(self.__send_target_sub)
            self._node.destroy_subscription(self.__takeoff_sub)
            self._node.destroy_subscription(self.__land_sub)
            self._node.destroy_timer(self.__info_timer)
            self._node.destroy_publisher(self.__info_pub)

        wait_timeout = 10.0 # Maximum wait time, before just stopping and deactivatig actor -> fail safe
        while self._state == PadFlieState.LAND and wait_timeout > 0.0: 
            """If we are landing. Wait for Landing to complete."""
            self._sleep(0.1)
            wait_timeout -= 0.1        
        while self._state == PadFlieState.TAKEOFF and wait_timeout > 0.0:
            """If we are taking off. We need to wait for the takeoff, we are then in target and can land."""
            self._sleep(0.1)
            wait_timeout -= 0.1
        if self._state == PadFlieState.TARGET:
            """If in target mode. Just land."""
            self.land()

        self._state = PadFlieState.DEACTIVATED
        self._actor.deactivate()

        if self.has_subscriptions: 
            self._release_pad_right() # Maybe we have it. If we dont it is fine.

            self._node.destroy_client(self.__acquire_pad_rights_client)
            self._node.destroy_client(self.__release_pad_rights_client)

        self.has_subscriptions = False

    def get_state(self) -> PadFlieState:
        return self._state
    
    def _info_timer_callback(self):
        msg = PadflieInfo()
        msg.cf_prefix = self._cf_prefix
        pose_world = self.get_cf_pose_world()
        pose = self.get_cf_pose_stamped()
        if pose_world is not None: 
            msg.pose_world = pose_world
            msg.pose_world_valid = True
        if pose is not None: 
            msg.pose = pose
            msg.pose_valid = True
        msg.is_home = self._state == PadFlieState.IDLE
        self.__info_pub.publish(msg)

    def get_cf_pose_world(self) -> Optional[Pose]:
        position = self._tf_manager.get_cf_position()
        if position is None:
            return None
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = Rotation.from_euler("z", self._actor.get_yaw()).as_quat()
        return pose
    
    def get_cf_pose_stamped(self) -> Optional[PoseStamped]:
        """Returns pose stamped of frame which send_target was last called with"""
        target_pose = self._actor.get_target_pose()
        if target_pose is None:
            return None
        frame_id = target_pose.header.frame_id
        quat = Quaternion()
        (
            quat.x,
            quat.y,
            quat.z,
            quat.w,
        ) = Rotation.from_euler("z", self._actor.get_yaw()).as_quat()
        return self._tf_manager.get_cf_pose_stamped(frame_id=frame_id,world_quat=quat)
    
    def _acquire_pad_right(self, timeout: float) -> Future:
        if self.__acquire_pad_rights_client.wait_for_service(timeout_sec=timeout):
            req = PadRightAcquire.Request()
            req.name = self._prefix
            req.timeout = timeout
            return self.__acquire_pad_rights_client.call_async(req)
        else: 
            fut = Future()
            resp = PadRightAcquire.Response()
            resp.success = True
            fut.set_result(resp)
            return fut
        
    def _release_pad_right(self) -> Future:
        if self.__release_pad_rights_client.wait_for_service(timeout_sec=1.0):
            req = PadRightRelease.Request()
            req.name = self._prefix
            return self.__release_pad_rights_client.call_async(req)
        else: 
            fut = Future()
            resp = PadRightRelease.Response()
            resp.success = True
            fut.set_result(resp)
            return fut
    
    def _set_target_internal(self, target: PoseStamped, use_yaw: bool = False):
        """Set a PoseStamped as the actor target.

        Args:
            target (PoseStamped): The target for the crazyflie.
            use_yaw (bool): Wheter to use the yaw in the pose (True) or set it to 0 (False)
        """
        self._actor.set_target(target, use_yaw)

    def send_target(self, target: PoseStamped, use_yaw: bool = False):
        """Sends a target to crazyflie if we are in target mode."""
        if self._state is not PadFlieState.TARGET:
            return
        self._set_target_internal(target, use_yaw)

    def takeoff(self):
        """Execute crazyflie takeoff. If in IDLE.

        The crazyflie will only takeoff if it knows its position.

        This routine takes exactly 2 seconds to complete

        """
        if self._state is not PadFlieState.IDLE:
            return
        
        def _takeoff(fut: Future):
            success: PadRightAcquire.Response = fut.result()
            if not success.success: 
                self._node.get_logger("Did not get takeoff rights. Staying Home.")
                return
            
            position = self._tf_manager.get_cf_position()
            if position is None:
                self._node.get_logger().error(
                    "Crazyflie doesnt have position. Cannot takeoff."
                )
                self._release_pad_right()
                return

            self._state = PadFlieState.TAKEOFF
            success = self._actor.takeoff_routine()
            self._state = PadFlieState.TARGET
            self._release_pad_right()
                
        fut = self._acquire_pad_right(timeout=60.0) 
        # Up to one minute. Hmm...
        fut.add_done_callback(_takeoff)

        
    def land(self):
        """Execute crazyflie landing. If in target mode.

        The crazyflie will only land if it knows its own position and the position of its landing pad.

        If the position and pad_position is valid is only checked once.
        After that landing will try to get updated positions but receives old ones if there is an issue.

        The routine transitions us through TARGET_INTERNAL, LAND and leaves with state IDLE
        It takes 16.5 seconds

        TODO: Maybe there are strategies to handle this better. But if there is no position available something is bad anyway.
        """
        if self._state is not PadFlieState.TARGET:
            return
    
        self._state = PadFlieState.LAND

        fut = self._acquire_pad_right(60.0)
        while not fut.done():
            # Fly in pad_circle during this time. 
            #self._node.get_logger().info("Waiting for Landing rights.")
            self._sleep(0.1) 
        success: PadRightAcquire.Response = fut.result()
        if not success.success: 
            self._state = PadFlieState.TARGET
        """
        Phase 1:
            For at most 8 seconds.
            Fly with safe targets to a point 0.5 meters above the pad.
            If position is reached continue with Phase2.

            If is is not reached PROBLEM TODO

            During this we need to update the pad position, maybe it moves.

            Phases 2 and 3 are in the routine.
        """
        self._node.get_logger().info("Land P1")
        
        timeout = 8.0
        while timeout > 0.0:
            target_pose = self._tf_manager.get_pad_pose()
            target_pose.pose.position.z += 0.5
            #self._node.get_logger().info(f"Setting internal target {target_pose}")     
            self._set_target_internal(target=target_pose, use_yaw=True)

            cf_position = self._tf_manager.get_cf_position()
            pad_target = self._tf_manager.pose_stamped_to_world_position_and_yaw(
                target_pose
            )
            if cf_position is None or pad_target is None:
                self._actor._fail_safe("Landing failed. CF Position or Pad position lost.")
                self._state = PadFlieState.DEACTIVATED
                self._release_pad_right()
                return

            target, _yaw = pad_target
            if (
                np.linalg.norm(np.array(cf_position) - np.array(target)) < 0.25
            ):  # closer than 25 cm radius
                break

            timeout -= 0.1
            self._sleep(0.1)
       
        self._node.get_logger().info("Land Routine")     
        self._actor.land_routine()
        self._state = PadFlieState.IDLE
        self._release_pad_right()

    def __send_target_callback(self, msg: SendTarget):
        """Callback to the send_target topic.

        Args:
            msg (SendTarget): A Send target message with priority and posestamped as target.
        """
        #if msg.priority_id >= self._current_priority_id:
        self.send_target(msg.target, msg.use_yaw)

    def __takeoff_callback(self, msg: Empty):
        """Callback to the takeoff subscription.

        Args:
            msg (Empty): An Empty message. Just used as trigger.
        """
        self._node.get_logger().info("Pad-Takeoff")
        self.takeoff()

    def __land_callback(self, msg: Empty):
        """Callback to the land subscription.

        Args:
            msg (Empty): An empty message. Just used as trigger.
        """
        self._node.get_logger().info("Pad-Land")
        self.land()
