import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from crazyflies.crazyflie import Crazyflie, CrazyflieType

from std_msgs.msg import Empty

from crazyflies_interfaces.msg import SendTarget
from crazyflies.safe.safe_commander import SafeCommander

from typing import List
import signal
from enum import Enum, auto
from dataclasses import dataclass

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from copy import deepcopy


class PadFlieState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()


PAD_FLIE_TYPE = "tracked"


class PadFlie(Crazyflie):
    def __init__(self, node: Node, id: int, channel: int, pad_name: str):
        self.node = node
        self.pad_name = pad_name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        node.get_logger().info(f"{id},{channel},{pad_name}")

        timeout = 0
        pad_position = self._get_pad_position()
        while pad_position is None and not timeout > 100:
            rclpy.spin_once(
                node, timeout_sec=0.05
            )  ## Spin abit to get listener.. only for now
            pad_position = self._get_pad_position()
            timeout += 1
        if timeout > 100:
            node.get_logger().info("nope")

        super().__init__(node, id, channel, pad_position, type=CrazyflieType.HARDWARE)
        self.state: PadFlie = PadFlieState.IDLE

        prefix = "/padflie{}".format(id)
        qos_profile = 10
        callback_group = MutuallyExclusiveCallbackGroup()

        self.target: List[float] = None

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

        update_rate = 10.0  # Hz
        dt = 1 / update_rate

        self.commander = SafeCommander(
            dt=dt, max_step_distance_xy=3, max_step_distance_z=1, clipping_box=None
        )

        cmd_position_timer = self.node.create_timer(
            dt, self.__send_target, callback_group=MutuallyExclusiveCallbackGroup()
        )

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
        self.target[2] += 1
        self._sleep(4.0)

        self.state = PadFlieState.LAND
        self.land(target_height=pad_position[2], duration_seconds=4.0)
        self._sleep(duration=4.0)
        self.state = PadFlieState.IDLE

    def _sleep(self, duration: float) -> None:
        """Sleeps for the provided duration in seconds."""
        start = self.__time()
        end = start + duration
        while self.__time() < end:
            rclpy.spin_once(self.node, timeout_sec=0)

    def __time(self) -> "Time":
        """Return current time in seconds."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def _get_pad_position(self) -> List[float]:
        try:
            t = self.tf_buffer.lookup_transform(
                "world", self.pad_name, rclpy.time.Time()
            )
            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
        except Exception as ex:
            self.node.get_logger().info(str(ex))
            return None


def main():
    rclpy.init()
    node = Node("padflie")
    node.declare_parameter(
        name="id", value=0xE7, descriptor=ParameterDescriptor(read_only=True)
    )
    node.declare_parameter(
        name="channel", value=80, descriptor=ParameterDescriptor(read_only=True)
    )
    node.declare_parameter(
        name="pad_name",
        value="pad_",
        descriptor=ParameterDescriptor(read_only=True),
    )

    cf_id: int = node.get_parameter("id").get_parameter_value().integer_value
    cf_channel: int = node.get_parameter("channel").get_parameter_value().integer_value
    pad_name: int = node.get_parameter("pad_name").get_parameter_value().string_value

    safeflie = PadFlie(node, cf_id, cf_channel, pad_name)

    @dataclass
    class FLAG:
        stop: bool = False

        def kill(self):
            self.stop = True

    SHUTDOWN = FLAG()
    signal.signal(signal.SIGINT, lambda _, __: SHUTDOWN.kill())

    while rclpy.ok() and not SHUTDOWN.stop:
        rclpy.spin_once(node, timeout_sec=0.1)

    safeflie.close_crazyflie()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
