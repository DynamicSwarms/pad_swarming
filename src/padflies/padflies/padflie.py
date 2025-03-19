import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, Executor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.lifecycle import LifecycleNodeMixin
from rclpy.lifecycle import State, TransitionCallbackReturn

from lifecycle_msgs.msg import State as LifecycleState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from rcl_interfaces.srv import SetParameters


import rclpy.parameter


from crazyflies.crazyflie import CrazyflieType, Crazyflie
from crazyflies.gateway_endpoint import CrazyflieGatewayError
from .controller import PadflieController

from ._padflie_tf import PadflieTF

import traceback
import signal
from dataclasses import dataclass

from typing import List, Optional, Tuple, Union


from crazyflie_interfaces_python.client import (
    ConsoleClient,
    GenericCommanderClient,
    HighLevelCommanderClient,
    LoggingClient,
)
from crazyflies.gateway_endpoint import GatewayEndpoint


PAD_FLIE_TYPE = "tracked"


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
        self.declare_parameter(
            name="type",
            value="hardware",
            descriptor=ParameterDescriptor(read_only=True),
        )

    def configure(self) -> bool:
        self.get_logger().debug(
            f"PadFlie ID:{self.cf_id}, CH:{self.cf_channel}, PAD:{self.pad_name}, Type {self.cf_type} configuring."
        )
        
        
        self._prefix = "/padflie{}".format(self.cf_id)
        cf_prefix = "/cf{}".format(self.cf_id)
        self._cf_prefix = cf_prefix

        self._tf_manager = PadflieTF(
            node=self,
            sleep=self._sleep,
            pad_name=self.pad_name,
            cf_name=f"cf{self.cf_id}",
            world="world",
        )

        pad_position, yaw = self._tf_manager.get_pad_position_or_timeout(
            timeout_sec=1.0
        )  # might raise TimeoutError

        loginfo = lambda msg: self.get_logger().info(str(msg))
        self.console = ConsoleClient(node=self, prefix=cf_prefix, callback=loginfo)
        
        self.get_logger().info("Padflie configure deactivated for performance")
        return True

        self._sleep(4.0)  # Make sure all crazyflie services are initialized properly

        self.set_initial_yaw()

        self._commander = PadflieController(
            node=self,
            cf_type=self.cf_type,
            prefix=self._prefix,
            hl_commander=HighLevelCommanderClient(node=self, prefix=cf_prefix),
            ll_commander=GenericCommanderClient(node=self, prefix=cf_prefix),
            log_commander=LoggingClient(node=self, prefix=cf_prefix),
            tf_manager=self._tf_manager,
            sleep=self._sleep,
        )  # This starts the main control loop of the padflie

        return True

    def set_initial_yaw(self):
        p_y = self._tf_manager.get_pad_position_and_yaw()
        if p_y is None:
            return

        _, yaw = p_y
        self.set_parameter("kalman.initialYaw", float(yaw))
        self.set_parameter("kalman.resetEstimation", 1)

    def set_parameter(self, name: str, value: Union[int, float]):
        client = self.create_client(SetParameters, f"{self._cf_prefix}/set_parameters")

        if client.service_is_ready():
            req = SetParameters.Request()
            param = Parameter()
            param.name = name

            if isinstance(value, int):
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = value
            elif isinstance(value, float):
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value

            req.parameters.append(param)
            client.call_async(req)

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

    @property
    def cf_type(self) -> CrazyflieType:
        tp = self.get_parameter("type").get_parameter_value().string_value
        return CrazyflieType.HARDWARE if tp == "hardware" else CrazyflieType.WEBOTS

    # Lifecycle Overrides
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_configure(self, state)

        try:
            if self.configure():
                self.get_logger().info("Configuring complete")
                return TransitionCallbackReturn.SUCCESS
            else:
                self.get_logger().info("Configuring failed")
                return TransitionCallbackReturn.FAILURE
        except (TimeoutError, CrazyflieGatewayError) as ex:
            self.get_logger().info(
                "Crazyflie configuration failed, because: "
                + str(ex)
                + " Transitioning to unconfigured."
            )
            return TransitionCallbackReturn.FAILURE  # Going back to unconfigured
        except Exception as ex:
            # The lifecycle implementation catches Errors but does not process them and doesnt warn user.
            # We therefore catch them here and present user feedback
            self.get_logger().info(str(traceback.format_exc()))
            return TransitionCallbackReturn.ERROR

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

    def on_error(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_error(self, state)
        self.get_logger().info(f"PadFlie {self.cf_id} processing error.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        LifecycleNodeMixin.on_cleanup(self, state)
        self.get_logger().info(f"Padflie {self.cf_id} cleaning up")
        return TransitionCallbackReturn.SUCCESS

    def shutdown(self):
        if (
            self._state_machine.current_state[0]
            is not LifecycleState.PRIMARY_STATE_UNCONFIGURED
        ):
            try:
                self.gateway_endpoint.close()
                self._sleep(1.0)
            except CrazyflieGatewayError as ex:
                # The Gateway might have shutdown already.
                # This is fine.
                self.get_logger().info(str(ex) + " (Expected on ctrl+c)")
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
        rclpy.spin_once(padflie, executor=executor, timeout_sec=1.0)

    padflie.shutdown()
    padflie.destroy_node()
    rclpy.try_shutdown()
    exit()


if __name__ == "__main__":
    main()
