import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, Executor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    SetParametersResult,
    FloatingPointRange,
)

from geometry_msgs.msg import PoseStamped

from agents.connector import AgentConnector
from agents.commander import AgentCommander
import signal


from typing import List
from dataclasses import dataclass


class WandAgent(Node):

    def __init__(self, executor: Executor):
        super().__init__("agent")
        self.declare_parameter(
            name="id", value=0, descriptor=ParameterDescriptor(read_only=True)
        )
        self.declare_parameter(name="active", value=False)
        self.add_on_set_parameters_callback(self.__set_parameter_callback)

        self.__commander = AgentCommander(node=self)
        self.__connector = AgentConnector(
            node=self,
            executor=executor,
            on_connect_callback=self.on_connect,
            on_disconnect_callback=self.on_disconnect,
        )

        ## Wand Agent Code
        self.declare_parameter("wand", value="Wand1")
        self.declare_parameter(
            "distance",
            value=0.5,
            descriptor=ParameterDescriptor(
                floating_point_range=[
                    FloatingPointRange(from_value=0.3, to_value=2.0),
                ]
            ),
        )

        self.create_timer(1.0, self.on_timer)

    def on_connect(self, prefix: str, p_id: int):
        self.__commander.on_connect(prefix=prefix, priority_id=p_id)
        self.__commander.takeoff()

    def on_disconnect(self):
        self.__commander.land()
        self.__commander.on_disconnect()

    def on_timer(self):
        target = PoseStamped()
        target.header.frame_id = self.wand
        target.pose.position.x = self.distance
        self.__commander.send_target(target)

        self.get_logger().info(f"{self.__commander.get_local_position()}")

    @property
    def wand(self) -> str:
        return self.get_parameter("wand").get_parameter_value().string_value

    @property
    def distance(self) -> str:
        return self.get_parameter("distance").get_parameter_value().double_value

    def trigger_activate(self, activate: bool):
        self.__connector.set_active(activate)

    def __set_parameter_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == "active":
                self.trigger_activate(param.value)
        return SetParametersResult(successful=True)

    @property
    def active(self) -> bool:
        return self.get_parameter("active").get_parameter_value().bool_value

    def shutdown(self):
        if self.__connector.is_connected():
            self.__connector.disconnect()
            # Because this triggers on_disconnect callback the CF will land.
            # I do not know if this is expected bahaviour
        else:
            self.get_logger().info(f"Not connected {self.__connector.is_connected()}")
        self.executor.shutdown(timeout_sec=0.1)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    ag = WandAgent(executor)

    @dataclass
    class FLAG:
        stop: bool = False

        def kill(self):
            self.stop = True

    SHUTDOWN = FLAG()
    signal.signal(signal.SIGINT, lambda _, __: SHUTDOWN.kill())

    while rclpy.ok() and not SHUTDOWN.stop and not executor._is_shutdown:
        rclpy.spin_once(ag, timeout_sec=0.1, executor=executor)

    ag.shutdown()
    ag.destroy_node()
    rclpy.try_shutdown()
    exit()


if __name__ == "__main__":
    main()
