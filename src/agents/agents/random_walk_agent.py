import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, Executor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from padflies_interfaces.msg import AvailabilityInfo
from padflies_interfaces.srv import Connect
from std_srvs.srv import Empty

import signal


from typing import Optional
from dataclasses import dataclass


class RandomWalkAgent(Node):

    def __init__(self, executor: Executor):
        super().__init__("agent")
        self.declare_parameter(
            name="id", value=0, descriptor=ParameterDescriptor(read_only=True)
        )
        self.__connected = False
        self._executor = executor

        self.__disconnect_service_name: Optional[str] = None

        # Create a QoS profile for maximum performance
        qos_profile_performance = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Minimal latency, no retries
            durability=DurabilityPolicy.VOLATILE,  # Only delivers data to currently available subscribers
            history=HistoryPolicy.KEEP_LAST,  # Keeps only the last N messages
            depth=1,  # Keeps a short history to reduce memory use
        )

        self.create_subscription(
            msg_type=AvailabilityInfo,
            topic="availability",
            callback=self.__availability_callback,
            qos_profile=qos_profile_performance,
        )

    def __availability_callback(self, info: AvailabilityInfo):
        if self.__connected:
            return
        proxy = self.create_client(
            srv_type=Connect,
            srv_name=info.connect_service_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        req = Connect.Request()
        fut = proxy.call_async(req)
        rclpy.spin_until_future_complete(
            self, fut, executor=self.executor, timeout_sec=0.1
        )
        response: Optional[Connect.Response] = fut.result()

        if response is not None and response.success:
            self.__connected = True
            self.__disconnect_service_name = info.disconnect_service_name
            self.get_logger().info(
                f"Connected to {info.connect_service_name}, {self.__connected}"
            )

    def shutdown(self):
        if self.__connected:
            if self.__disconnect_service_name is not None:
                proxy = self.create_client(
                    srv_type=Empty,
                    srv_name=self.__disconnect_service_name,
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
                req = Empty.Request()
                fut = proxy.call_async(req)
                rclpy.spin_until_future_complete(
                    self, fut, executor=self.executor, timeout_sec=0.1
                )
                self.get_logger().info(f"Disconnected!")
            else:
                self.get_logger().info(f"Disconnect failed. Not srv name!")
        else:
            self.get_logger().info(f"Not connected {self.__connected}")
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
