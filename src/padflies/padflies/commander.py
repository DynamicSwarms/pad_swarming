from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Empty
from crazyflies_interfaces.msg import SendTarget
from padflies_interfaces.srv import Connect
from padflies_interfaces.msg import AvailabilityInfo

from crazyflie_interfaces_python.client import (
    HighLevelCommanderClient,
    GenericCommanderClient,
)
from crazyflies.safe.safe_commander import SafeCommander

from .charge_controller import ChargeController
from .padflie import PadFlie
from .routines import takeoff_routine, land_routine

from enum import Enum, auto

from typing import Callable, List, Optional


class PadFlieState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    LAND = auto()
    TARGET = auto()
    TARGET_INTERNAL = auto()  # Do not accept external targets. But flie with target


class PadflieCommander:
    def __init__(
        self,
        padflie: PadFlie,
        prefix: str,
        hl_commander: HighLevelCommanderClient,
        g_commander: GenericCommanderClient,
        get_position_callback: Callable[[], Optional[List[float]]],
        get_pad_position_callback: Callable[[], Optional[List[float]]],
        sleep_callback: Callable[[float], None],
    ):
        self.hl_commander = hl_commander
        self.ll_commander = g_commander
        self.get_position: Callable[[], Optional[List[float]]] = get_position_callback
        self.get_pad_position: Callable[[], Optional[List[float]]] = (
            get_pad_position_callback
        )
        self.sleep: Callable[[float],] = sleep_callback

        self.state: PadFlieState = PadFlieState.IDLE
        self.target: Optional[List[float]] = None
        self.connected: bool = False

        self.charge_controller = ChargeController(self, self)

        availability_rate: float = 1.0

        target_rate: float = 10.0  # Hz
        dt: float = 1 / target_rate
        self.commander = SafeCommander(
            dt=dt, max_step_distance_xy=3, max_step_distance_z=1, clipping_box=None
        )

        padflie.create_timer(
            timer_period_sec=dt,
            callback=self._send_target,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )  # This timer needs to be executed while we takeoff or land -> different callback group

        callback_group = (
            MutuallyExclusiveCallbackGroup()
        )  # All subscriptions can be on the same callbackgroup
        qos_profile = 10
        padflie.create_subscription(
            SendTarget,
            prefix + "/send_target",
            self._send_target_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        padflie.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_takeoff",
            callback=self.__takeoff_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        padflie.create_subscription(
            msg_type=Empty,
            topic=prefix + "/pad_land",
            callback=self.__land_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
        )

        padflie.create_service(
            srv_type=Connect,
            srv_name=prefix + "/connect",
            callback=self.handle_connect_request,
            callback_group=callback_group,
        )

        # Create a QoS profile for maximum performance
        qos_profile_performance = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Minimal latency, no retries
            durability=DurabilityPolicy.VOLATILE,  # Only delivers data to currently available subscribers
            history=HistoryPolicy.KEEP_LAST,  # Keeps only the last N messages
            depth=1,  # Keeps a short history to reduce memory use
        )
        self.availability_publisher: Publisher = padflie.create_publisher(
            msg_type=AvailabilityInfo,
            topic="availability",
            qos_profile=qos_profile_performance,
            callback_group=callback_group,
        )

        padflie.create_timer(
            timer_period_sec=availability_rate,
            callback=self.send_availability_info,
            callback_group=callback_group,
        )

    def handle_connect_request(
        self, request: Connect.Request, response: Connect.Response
    ):
        if self.state == PadFlieState.IDLE and not self.connected:
            self.connected = True
            response.success = True
        else:
            response.success = False
        return response

    def send_availability_info(self):
        # Publish information about us.
        # Including position , ready state etc. onto global topic.
        msg = AvailabilityInfo()
        msg.available = self.state == PadFlieState.IDLE and not self.connected
        self.availability_publisher.publish(msg)

    def _send_target(self):
        if self.state not in (PadFlieState.TARGET, PadFlieState.TARGET_INTERNAL):
            return
        position = self.get_position()
        if position is not None and self.target is not None:
            safe_target = self.commander.safe_cmd_position(position, self.target)
            self.ll_commander.cmd_position(safe_target, 0.0)

    def _set_target(self, target: List[float]) -> None:
        self.target = target

    def _send_target_callback(self, msg: SendTarget) -> None:
        if self.state is not PadFlieState.TARGET_INTERNAL:
            self._set_target([msg.target.x, msg.target.y, msg.target.z])

    def __takeoff_callback(self, msg: Empty) -> None:
        if self.state is not PadFlieState.IDLE:
            return

        position = self.get_position()
        if position is None:
            raise Exception("Crazyflie doesnt have position. Cannot takeoff.")

        takeoff_routine(position)
        # This routine takes exactly 2 seconds to complete

    def __land_callback(self, msg: Empty) -> None:
        if self.state not in (PadFlieState.TARGET, PadFlieState.TARGET_INTERNAL):
            return

        pad_position: Optional[List[float]] = self.get_pad_position()
        position: Optional[List[float]] = self.get_position()

        # Failsafe if this is None??
        if pad_position is None:
            raise Exception("Could not find pad. Cannot land")
        if position is None:
            raise Exception("Was not able to find the position.")

        def get_pad_position():
            new_pad_position = self.get_pad_position()
            if new_pad_position is not None:
                pad_position = new_pad_position
                return pad_position
            return pad_position

        def get_position():
            new_position = self.get_position()
            if new_position is not None:
                position = new_position
            return position

        land_routine(
            commander=self, get_pad_position=get_pad_position, get_position=get_position
        )
        # This routine transitions us through TARGET_INTERNAL, LAND and leaves with state IDLE
        # It takes 16.5 seconds
