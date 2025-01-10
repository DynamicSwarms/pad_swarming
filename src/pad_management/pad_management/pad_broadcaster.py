import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml

from ament_index_python.packages import get_package_share_directory


class PadBroadcaster(Node):
    def __init__(self):
        super().__init__("pad_broadcaster")
        self.declare_parameter(
            "pad_yaml",
            get_package_share_directory("pad_management")
            + "/config/pad_arrangement.yaml",
        )
        self.declare_parameter("pad_size", 0.2)
        self.declare_parameter("base", "ChargingBase20")
        self.declare_parameter("rate_hz", 10)

        yaml_file = self.get_parameter("pad_yaml").get_parameter_value().string_value
        self.pad_size = (
            self.get_parameter("pad_size").get_parameter_value().double_value
        )
        self.base = self.get_parameter("base").get_parameter_value().string_value
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().integer_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Open and read the YAML file
        file = open(yaml_file, "r")
        self.pads = yaml.safe_load(file)["pads"]

        self.create_timer(1.0 / self.rate_hz, self.publish_pads)

    def publish_pads(self):
        for pad in self.pads:
            self.tf_broadcaster.sendTransform(self._transform_stamped_from_pad(pad))

    def _transform_stamped_from_pad(self, pad) -> TransformStamped:
        v = (self.pad_size * pad["pos"][0], self.pad_size * pad["pos"][1], 0.0)
        q = (0.0, 0.0, 0.0, 1.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base
        t.child_frame_id = "pad_{}".format(pad["id"])

        (
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ) = v
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = q

        return t


def main():
    rclpy.init()

    pb = PadBroadcaster()

    try:
        while rclpy.ok():
            rclpy.spin_once(pb, timeout_sec=0.1)
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        quit()


if __name__ == "__main__":
    main()
