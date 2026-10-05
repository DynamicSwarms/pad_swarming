"""Publish a scaled clock with 10 ms simulation-time resolution."""
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMessage


class SimulationClock(Node):
    def __init__(self):
        super().__init__('velocity_simulation_clock')
        speed = self.declare_parameter('speed', 1.0).value
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError('speed must be finite and positive')
        self.stamp = 0
        self.publisher = self.create_publisher(ClockMessage, '/clock', 10)
        self.timer = self.create_timer(0.01 / speed, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        self.stamp += 10_000_000
        msg = ClockMessage()
        msg.clock.sec, msg.clock.nanosec = divmod(self.stamp, 1_000_000_000)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SimulationClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
