import rclpy
from rclpy.node import Node
from pad_management_interfaces.srv import PointFinder
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import Point

from typing import Optional
from math import sqrt, pow
import numpy as np

from numpy.typing import NDArray


class PointCloudFinder(Node):

    def __init__(self) -> None:
        super().__init__("pc_finder")
        self.declare_parameter("point_cloud_topic_name", "point_cloud")
        pc_topic_name = (
            self.get_parameter("point_cloud_topic_name")
            .get_parameter_value()
            .string_value
        )

        self.point_cloud: Optional[PointCloud2] = None

        self.create_subscription(PointCloud2, pc_topic_name, self.pc_callback, 10)

        self.create_service(PointFinder, "~/checkForPoint", self.check_point)

    def check_point(self, request: PointFinder.Request, response: PointFinder.Response):
        response.found = self._check_for_point(request.point, request.max_distance)
        return response

    def pc_callback(self, msg: PointCloud2):
        self.point_cloud = msg

    def _check_for_point(self, point: Point, distance: float) -> bool:
        if self.point_cloud is None:
            return False

        p1 = np.asarray((point.x, point.y, point.z))
        for np_point in read_points(self.point_cloud, skip_nans=True):
            p2 = np.asarray([np_point["x"], np_point["y"], np_point["z"]])
            if self._distance_between_points(p1, p2) < distance:
                return True
        return False

    def _distance_between_points(self, p1: NDArray, p2: NDArray) -> float:
        return np.linalg.norm(p1 - p2)


def main():
    rclpy.init()

    pcf = PointCloudFinder()
    try:
        while rclpy.ok():
            rclpy.spin_once(pcf, timeout_sec=0.1)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
