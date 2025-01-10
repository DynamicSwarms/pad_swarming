import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points
from geometry_msgs.msg import Point

from typing import Optional, Dict
import numpy as np

from numpy.typing import NDArray


class PointCloudCombiner(Node):

    def __init__(self) -> None:
        super().__init__("pc_combiner")
        self.declare_parameter("input_names", ["pointCloud"])
        self.declare_parameter("output_name", "pointCloud2")

        output_name = (
            self.get_parameter("output_name").get_parameter_value().string_value
        )
        input_names = (
            self.get_parameter("input_names").get_parameter_value().string_array_value
        )

        self.point_clouds: Dict[str, PointCloud2] = {}

        for name in input_names:
            self.create_subscription(
                PointCloud2, name, lambda msg: self.pc_callback(msg, name), 10
            )

        self.pc_publisher = self.create_publisher(PointCloud2, output_name, 10)
        self.create_timer(1 / 100.0, self.publish_merge)

    def pc_callback(self, msg: PointCloud2, name: str):
        self.point_clouds[name] = msg

    def publish_merge(self):
        point_cloud_data = np.empty((0, 3), dtype=np.float32)
        world = "world"
        for point_cloud in self.point_clouds.values():
            points = read_points(point_cloud)
            point_cloud_data = np.concatenate(
                (
                    point_cloud_data,
                    np.column_stack((points["x"], points["y"], points["z"])),
                )
            )

        msg_point_cloud = PointCloud2()
        msg_point_cloud.header.stamp = self.get_clock().now().to_msg()
        msg_point_cloud.header.frame_id = world
        msg_point_cloud.width = point_cloud_data.shape[0]
        msg_point_cloud.height = 1
        msg_point_cloud.fields.append(
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)
        )
        msg_point_cloud.fields.append(
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)
        )
        msg_point_cloud.fields.append(
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        )
        msg_point_cloud.is_bigendian = False
        msg_point_cloud.point_step = 12
        msg_point_cloud.row_step = (
            msg_point_cloud.point_step * point_cloud_data.shape[0]
        )
        msg_point_cloud.is_dense = True
        msg_point_cloud.data = point_cloud_data.astype(np.float32).tobytes()
        self.pc_publisher.publish(msg_point_cloud)


def main():
    rclpy.init()

    pcc = PointCloudCombiner()
    try:
        while rclpy.ok():
            rclpy.spin_once(pcc, timeout_sec=0.1)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
