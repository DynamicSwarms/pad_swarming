import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNodeMixin
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.msg import State as LifecycleState

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor, Executor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from crazyflies.crazyflie import CrazyflieType, Crazyflie
from crazyflies.gateway_endpoint import GatewayEndpoint, CrazyflieGatewayError

from std_msgs.msg import Empty

from crazyflies_interfaces.msg import SendTarget
from crazyflies.safe.safe_commander import SafeCommander

from typing import List
import signal
from enum import Enum, auto
from dataclasses import dataclass

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .charge_controller import ChargeController


from copy import deepcopy
import traceback


class PadFlieState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()


PAD_FLIE_TYPE = "tracked"
from crazyflie_hardware_gateway_interfaces.srv import Crazyflie as HardwareCrazyflie
from crazyflie_webots_gateway_interfaces.srv import WebotsCrazyflie

import numpy as np


class PadFlie(Node, LifecycleNodeMixin, Crazyflie):
    def __init__(self, executor: Executor):
        Node.__init__(self, node_name="padflie")
        LifecycleNodeMixin.__init__(
            self, callback_group=MutuallyExclusiveCallbackGroup()
        )  # The Services need to be in the not default Callback Group because TFBuffer will live there
        self.executor = executor
        self.declare_parameter(
            name="id", value=0xE7, descriptor=ParameterDescriptor(read_only=True)
        )
        self.declare_parameter(
            name="channel", value=80, descriptor=ParameterDescriptor(read_only=True)
        )
        self.declare_parameter(
            name="pad_id",
            value=0,
            descriptor=ParameterDescriptor(read_only=True),
        )

    def configure(self) -> bool:
        # tf_node = Node(f"padflie{self.cf_id}_tflistener", use_global_arguments=False)
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, tf_node, spin_thread=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        """ "
        This is getting stupid. This buffer needs to be inside init but cannot.
        The crazyflie constructor also creates such buffer...
        Should we not use Crazyflie but the gateway... not nice...
        Executors suck. Ros1 was so nice in this regard
        """
        self.get_logger().info(f"{self.cf_id},{self.cf_channel},{self.pad_name}")

        timeout = 1.0  # secs
        pad_position = self._get_pad_position()
        while pad_position is None and timeout > 0.0:
            self._sleep(0.1)
            timeout -= 0.1
            pad_position = self._get_pad_position()
        if pad_position is None:
            self.get_logger().info("Crazyflie configuration failed, pad not found.")
            return False

        try:
            Crazyflie.__init__(
                self,
                self,
                self.cf_id,
                self.cf_channel,
                pad_position,
                CrazyflieType.HARDWARE,
            )
        except:
            self.get_logger().info("Crazyflie initialization failed.")
            self.get_logger().info(str(traceback.format_exc()))
            return False

        self._sleep(4.0)
        self.charge_controller = ChargeController(self, self)

        self.state: PadFlie = PadFlieState.IDLE

        prefix = "/padflie{}".format(self.cf_id)
        qos_profile = 10
        callback_group = MutuallyExclusiveCallbackGroup()

        self.target: List[float] = None

        self.create_subscription(
            SendTarget,
            prefix + "/send_target",
            self._send_target_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            callback=self._takeoff_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        self.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            callback=self._land_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        update_rate = 10.0  # Hz
        dt = 1 / update_rate

        self.commander = SafeCommander(
            dt=dt, max_step_distance_xy=3, max_step_distance_z=1, clipping_box=None
        )
        cmd_position_timer = self.create_timer(
            dt, self.__send_target, callback_group=MutuallyExclusiveCallbackGroup()
        )
        return True

    def __send_target(self):
        if self.state is not PadFlieState.TARGET:
            return
        position = self.get_position()
        if position is not None and self.target is not None:
            safe_target = self.commander.safe_cmd_position(position, self.target)
            self.cmd_position(safe_target, 0.0)

    def _send_target_callback(self, msg: SendTarget) -> None:
        x, y, z = msg.target.x, msg.target.y, msg.target.z
        self.target = [x, y, z]

    def _takeoff_callback(self, msg: Empty) -> None:
        position = self.get_position()
        if position is not None:
            self.target = position
            self.target[2] += 1.0
            self.state = PadFlieState.TAKEOFF
            self.takeoff(target_height=self.target[2], duration_seconds=4.0)
            self._sleep(4.0)
            self.state = PadFlieState.TARGET
        else:
            raise Exception("Crazyflie doesnt have position. Cannot takeoff.")

    def _land_callback(self, msg: Empty) -> None:
        pad_position = self._get_pad_position()
        if pad_position is None:
            raise Exception(
                "Could not find pad. Cannot land"
            )  # TODO: Dont crash but failsafe
        # Go above pad
        self.target = deepcopy(pad_position)
        self.target[2] += 0.5

        pos = self.get_position()
        timeout = 8.0
        while timeout > 0.0:
            if (
                pos is not None
                and np.linalg.norm(np.array(pos) - np.array(self.target)) < 0.1
            ):  # closer than 5 cm
                self.get_logger().info("Yes")
                break

            timeout -= 0.1
            self._sleep(0.1)
            pos = self.get_position()
            if pos is None:
                self.get_logger().info("Pos failed")

        self.state = PadFlieState.LAND
        self.notify_setpoints_stop(100)

        self.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2] + 0.2,
            yaw=0.0,
            duration_seconds=2.0,
        )
        # TODO: Make yaw available
        self._sleep(2.0)  # momentum
        self.go_to(
            pad_position[0],
            pad_position[1],
            pad_position[2],
            yaw=0.0,
            duration_seconds=4.0,
        )
        self._sleep(3.0)

        self.land(target_height=pad_position[2], duration_seconds=2.0)
        self._sleep(duration=2.0)
        self.state = PadFlieState.IDLE

    def _get_pad_position(self) -> List[float]:
        try:
            if not self.tf_buffer.can_transform_core(
                "world", self.pad_name, rclpy.time.Time()
            )[0]:
                return None

            t = self.tf_buffer.lookup_transform(
                target_frame="world",
                source_frame=self.pad_name,
                time=rclpy.time.Time(),
                # timeout=rclpy.duration.Duration(1.0),
            )
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception as ex:
            self.node.get_logger().info(str(ex))
            return None

    # Properties
    @property
    def cf_id(self) -> int:
        return self.get_parameter("id").get_parameter_value().integer_value

    @property
    def cf_channel(self) -> int:
        return self.get_parameter("channel").get_parameter_value().integer_value

    @property
    def pad_name(self) -> str:
        id: int = self.get_parameter("pad_id").get_parameter_value().integer_value
        return f"pad_{id}"

    # Lifecycle Overrides
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_configure(self, state)
        self.get_logger().info(f"PadFlie {self.cf_id} configuring.")
        try:
            if self.configure():
                self.get_logger().info("success")
                return TransitionCallbackReturn.SUCCESS
            else:
                self.get_logger().info("failed")
                return TransitionCallbackReturn.FAILURE
        except Exception as ex:
            # The transition callback catches exceptions but ommit them
            # We need to catch ourselfs, othewise no error is raised.

            self.get_logger().info("Exception in Configure")
            self.get_logger().info(str(traceback.format_exc()))
            self.get_logger().info("Exception trace Done, returning Failure!")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_activate(self, state)
        self.get_logger().info(f"PadFlie {self.cf_id} transitioned to active.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_deactivate(self, state)
        self.get_logger().info(f"PadFlie {self.cf_id} deactivating (not implemented)")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_shutdown(self, state)
        self.get_logger().info(f"PadFlie {self.cf_id} shutting down.")
        self.shutdown()
        return TransitionCallbackReturn.SUCCESS

    def shutdown(self):
        if (
            self._state_machine.current_state[0]
            is not LifecycleState.PRIMARY_STATE_UNCONFIGURED
        ):
            self.close_crazyflie()
            self._sleep(1.0)
            self.get_logger().info("here")

        self.executor.shutdown(timeout_sec=0.1)

    def _sleep(self, duration: float) -> None:
        """Sleeps for the provided duration in seconds."""
        start = self.__time()
        end = start + duration
        while self.__time() < end:
            rclpy.spin_once(self, timeout_sec=0, executor=self.executor)

    def __time(self) -> "Time":
        """Return current time in seconds."""
        return self.get_clock().now().nanoseconds / 1e9


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    padflie = PadFlie(executor)

    @dataclass
    class FLAG:
        stop: bool = False

        def kill(self):
            self.stop = True

    SHUTDOWN = FLAG()
    signal.signal(signal.SIGINT, lambda _, __: SHUTDOWN.kill())

    while rclpy.ok() and not SHUTDOWN.stop and not executor._is_shutdown:
        rclpy.spin_once(padflie, timeout_sec=0.1, executor=executor)

    padflie.shutdown()
    padflie.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
