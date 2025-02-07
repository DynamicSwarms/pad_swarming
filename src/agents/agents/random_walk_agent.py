import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, Executor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from agents.connector import AgentConnector
from agents.commander import AgentCommander
import signal


from typing import List
from dataclasses import dataclass


class RandomWalkAgent(Node):

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
            on_connect_callback=self.__commander.on_connect,
            on_disconnect_callback=self.__commander.on_disconnect,
        )

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
        else:
            self.get_logger().info(f"Not connected {self.__connector.is_connected()}")
        self.executor.shutdown(timeout_sec=0.1)


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    ag = RandomWalkAgent(executor)

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
