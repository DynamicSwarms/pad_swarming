import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

from pad_management_interfaces.srv import AcquireCrazyflie

from ament_index_python.packages import get_package_share_directory

import yaml

from typing import List, Optional

from crazyflie_interfaces.msg import Takeoff, Land, GoTo


class PadDestruction(Node):


    def __init__(self):
        super().__init__("pad_destruction")
        self.killed_cfs = []


        self.create_timer(1.0, self.destruct_test)
    
    def destruct_test(self):
        node_names = self.get_node_names()  # Get all active node names
        cf_nodes = [name for name in node_names if (name.startswith("cf") and not name.endswith("ctrl"))] # webots starts two nodes

        for cf_node in cf_nodes:
            if cf_node in self.killed_cfs: continue
            
            self.get_logger().info(f"Destructing: {cf_node}")
            takeoff = self.create_publisher(Takeoff,f"{cf_node}/takeoff",  10)
            while (takeoff.get_subscription_count() == 0):
                self._sleep(1.0)
                self.get_logger().info(f"Waiting for tk")
                
            tk = Takeoff()
            tk.duration.sec = 4
            tk.height = 1.5
            takeoff.publish(tk)
            self._sleep(3)

            go_to = self.create_publisher(GoTo,f"{cf_node}/go_to",  10)
            while (go_to.get_subscription_count() == 0):
                self._sleep(1.0)
                self.get_logger().info(f"Waiting for gt")

            gt = GoTo()
            gt.duration.sec = 4
            gt.goal.x = 2.0
            gt.goal.y = 0.0
            gt.goal.z = 0.0
            gt.relative = True
            go_to.publish(gt)
            self._sleep(4)

            land = self.create_publisher(Land, f"{cf_node}/land", 10)
            while (land.get_subscription_count() == 0):
                self._sleep(1.0)
                self.get_logger().info(f"Waiting for ld")
            ld = Land()
            ld.height = 0.0
            ld.duration.sec = 4
            land.publish(ld)
            self._sleep(1)

            self.killed_cfs.append(cf_node)
            return

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

    pd = PadDestruction()
    executor = SingleThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(node=pd, timeout_sec=0.1, executor=executor)
        rclpy.shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
